# Creates (or recreates) and fully builds the pluto HDL project from source,
# then exports the hardware platform (with bitstream) needed for the FSBL.
# Run from hdl/projects/pluto/ (build_all.sh cd's there first).

# Use every core the machine has rather than a hardcoded 8.
set njobs 8
if {![catch {exec nproc} n]} { set njobs $n }
puts "build_hdl: using $njobs parallel jobs"

set proj_exists [file exists pluto.xpr]

if {$proj_exists} {
    open_project pluto.xpr
    upgrade_ip [get_ips] -log ip_upgrade.log
    reset_run synth_1
    launch_runs synth_1 -jobs $njobs
    wait_on_run synth_1

    # ADI enables bitstream compression inside adi_project_run (see
    # projects/scripts/adi_project_xilinx.tcl), which only ever executes on the
    # project-CREATION path below. Reusing an existing pluto.xpr skips it, so
    # without this the same design emits an uncompressed bitstream and a
    # BOOT.bin ~1.7 MB larger (4.57 MB vs 2.85 MB) than a fresh build produces.
    # It boots either way, but output size should depend on the design, not on
    # whether the project happened to exist.
    #
    # Setting the property here on an opened synth_1 does NOT work, though it
    # looks like it should and is what ADI's own code appears to do: launch_runs
    # starts a SEPARATE Vivado process that re-reads the synthesis checkpoint
    # from disk, so an in-memory design property never reaches write_bitstream.
    # That was measured - the bitstream came out uncompressed with no warning.
    # A pre-hook on the write_bitstream step does run inside that process, with
    # the design open, which is the one place the property reliably applies.
    if {![info exists ::env(ADI_NO_BITSTREAM_COMPRESSION)]} {
        set compress_hook [file normalize ./set_bitstream_compress.tcl]
        set fh [open $compress_hook w]
        puts $fh {set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]}
        close $fh
        # Register the hook in utils_1 as well. Vivado runs it either way, but
        # warns (Runs 36-537) and leaves it out of project archives otherwise.
        if {[lsearch -exact [get_files -quiet -of_objects [get_filesets utils_1]] $compress_hook] < 0} {
            add_files -fileset utils_1 -norecurse $compress_hook
        }
        set_property STEPS.WRITE_BITSTREAM.TCL.PRE $compress_hook [get_runs impl_1]
    }

    launch_runs impl_1 -to_step write_bitstream -jobs $njobs
    wait_on_run impl_1
    open_run impl_1
} else {
    source system_project.tcl
    open_run impl_1
}

write_hw_platform -fixed -include_bit -force -file system_top.xsa
report_utilization -file utilization.rpt
report_timing_summary -file timing.rpt
exit

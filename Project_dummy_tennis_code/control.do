# Set the working dir, where all compiled Verilog goes.
vlib work

# Compile all verilog modules in lab4part1.v to working dir;
# could also have multiple verilog files.
vlog part2(3).v

# Load simulation using eightbitR as the top level simulation module.
vsim dummy_tennis_sim2

# Log all signals and add some signals to waveform window.
log {/*}
# add wave {/*} would add all items in top level simulation module.
add wave {/*}

# First test case
# Set input values using the force command, signal names need to be in {} brackets.

force {KEY[0]} 0 0, 1 25
force {SW[0]} 0 0, 1 25 -repeat 50 
force {SW[9]} 0 0, 1 25 -repeat 50
force {CLOCK_50} 0 0, 1 1 -repeat 2
# Run simulation for a few ns.
run 1000000ns


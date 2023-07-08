`include "top.v"

module testbench;
	reg clk;
	reg reset;
	wire [31:0] WriteData;
	wire [31:0] DataAdr;
	wire MemWrite;
	top dut(
		.clk(clk),
		.reset(reset),
		.WriteData(WriteData),
		.DataAdr(DataAdr),
		.MemWrite(MemWrite)
	);
	initial begin
		reset <= 1;
		#(22)
			;
		reset <= 0;
	end
	always begin
		clk <= 1;
		#(5)
			;
		clk <= 0;
		#(5)
			;
	end

	initial begin
		$dumpfile("lab5_1.vcd");
		$dumpvars(0);
	end

	always @(negedge clk) begin
		
		begin
			$display("dataAdr: %h   WriteData: %h, MemWrite: %h", DataAdr, WriteData, MemWrite);
		end


		if (MemWrite) begin
			if ((DataAdr === 100) & (WriteData === 7)) begin
				$display("Simulation succeeded");
				$stop;
			end
			else if (DataAdr !== 96) begin
				$display("Simulation failed");
				$stop;
			end
		end
	end
endmodule

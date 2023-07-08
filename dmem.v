module dmem (
	clk,
	we,
	a,
	wd,
	rd,
	oneb
);
	input wire clk;
	input wire we;
	input wire [31:0] a;
	input wire [31:0] wd;
	input wire oneb;
	output wire [31:0] rd;

	reg [31:0] RAM [63:0];
	assign rd = RAM[a[31:2]];
	always @(posedge clk)
		if (we)
			if (oneb)
				RAM[a[31:2]] <= (wd << 24) >> 24;
			else
				RAM[a[31:2]] <= wd;
endmodule

module adder_1_bit (
    input a,input b,input cin, output sum,output cout
    );
    xor(sum,a,b,cin);
    wire f1,f2,f3;
    and(f1,a,b);
    and(f2,a,cin);
    and(f3,b,cin);
    or(cout,f1,f2,f3);
endmodule

module adder_8_bit(
    input [7:0] a,b,
    input cin,
    output [7:0] sum,
    output cout
    );
    wire [6:0] c;
    adder_1_bit adder0(a[0],b[0],cin,sum[0],c[0]);
    adder_1_bit adder1(a[1],b[1],c[0],sum[1],c[1]);
    adder_1_bit adder2(a[2],b[2],c[1],sum[2],c[2]);
    adder_1_bit adder3(a[3],b[3],c[2],sum[3],c[3]);
    adder_1_bit adder4(a[4],b[4],c[3],sum[4],c[4]);
    adder_1_bit adder5(a[5],b[5],c[4],sum[5],c[5]);
    adder_1_bit adder6(a[6],b[6],c[5],sum[6],c[6]);
    adder_1_bit adder7(a[7],b[7],c[6],sum[7],cout);
endmodule


module adder_32_bit(
    input [31:0] a,b,
    input cin,
    output [31:0] sum
    );
    wire [2:0] c;
    wire cout;
    adder_8_bit adder0(a[7:0],b[7:0],cin,sum[7:0],c[0]);
    adder_8_bit adder1(a[15:8],b[15:8],c[0],sum[15:8],c[1]);
    adder_8_bit adder2(a[23:16],b[23:16],c[1],sum[23:16],c[2]);
    adder_8_bit adder3(a[31:24],b[31:24],c[2],sum[31:24],cout);
endmodule


module xor_8_bit(
    input [7:0] a,b,
    output [7:0] sum
    );
    xor xor0(sum[0],a[0],b[0]);
    xor xor1(sum[1],a[1],b[1]);
    xor xor2(sum[2],a[2],b[2]);
    xor xor3(sum[3],a[3],b[3]);
    xor xor4(sum[4],a[4],b[4]);
    xor xor5(sum[5],a[5],b[5]);
    xor xor6(sum[6],a[6],b[6]);
    xor xor7(sum[7],a[7],b[7]);
endmodule

module xor_32_bit(
    input [31:0] a,b,
    output [31:0] sum
    );
    xor_8_bit xor0(a[7:0],b[7:0],sum[7:0]);
    xor_8_bit xor1(a[15:8],b[15:8],sum[15:8]);
    xor_8_bit xor2(a[23:16],b[23:16],sum[23:16]);
    xor_8_bit xor3(a[31:24],b[31:24],sum[31:24]);
endmodule

module subtracter_32_bit(
    input [31:0] a,b,
    output [31:0] sum
    );
    wire [31:0] b_;
    parameter ONES = 32'b11111111_11111111_11111111_11111111;
    xor_32_bit xor0(ONES,b,b_);
    adder_32_bit adder0(a,b_,1'b1,sum);
endmodule


module and_8_bit(
    input [7:0] a,b,
    output [7:0] sum
    );
    and and0(sum[0],a[0],b[0]);
    and and1(sum[1],a[1],b[1]);
    and and2(sum[2],a[2],b[2]);
    and and3(sum[3],a[3],b[3]);
    and and4(sum[4],a[4],b[4]);
    and and5(sum[5],a[5],b[5]);
    and and6(sum[6],a[6],b[6]);
    and and7(sum[7],a[7],b[7]);
endmodule

module and_32_bit(
    input [31:0] a,b,
    output [31:0] sum
    );
    and_8_bit and0(a[7:0],b[7:0],sum[7:0]);
    and_8_bit and1(a[15:8],b[15:8],sum[15:8]);
    and_8_bit and2(a[23:16],b[23:16],sum[23:16]);
    and_8_bit and3(a[31:24],b[31:24],sum[31:24]);
endmodule

module or_8_bit(
    input [7:0] a,b,
    output [7:0] sum
    );
    or or0(sum[0],a[0],b[0]);
    or or1(sum[1],a[1],b[1]);
    or or2(sum[2],a[2],b[2]);
    or or3(sum[3],a[3],b[3]);
    or or4(sum[4],a[4],b[4]);
    or or5(sum[5],a[5],b[5]);
    or or6(sum[6],a[6],b[6]);
    or or7(sum[7],a[7],b[7]);
endmodule

module or_32_bit(
    input [31:0] a,b,
    output [31:0] sum
    );
    or_8_bit or0(a[7:0],b[7:0],sum[7:0]);
    or_8_bit or1(a[15:8],b[15:8],sum[15:8]);
    or_8_bit or2(a[23:16],b[23:16],sum[23:16]);
    or_8_bit or3(a[31:24],b[31:24],sum[31:24]);
endmodule

module not_8_bit(
    input [7:0] a,
    output [7:0] sum
    );
    not not0(sum[0],a[0]);
    not not1(sum[1],a[1]);
    not not2(sum[2],a[2]);
    not not3(sum[3],a[3]);
    not not4(sum[4],a[4]);
    not not5(sum[5],a[5]);
    not not6(sum[6],a[6]);
    not not7(sum[7],a[7]);
endmodule

module not_32_bit(
    input [31:0] a,
    output [31:0] sum
    );
    not_8_bit not0(a[7:0],sum[7:0]);
    not_8_bit not1(a[15:8],sum[15:8]);
    not_8_bit not2(a[23:16],sum[23:16]);
    not_8_bit not3(a[31:24],sum[31:24]);
endmodule


module SLA_32_bit(
    input [31:0] a,
    input shift,
    output [31:0] out
);
    assign out = a << shift;
endmodule

module SRA_32_bit(
    input signed [31:0] a,
    input shift,
    output signed [31:0] out
);
    assign out = a >>> shift;
endmodule

module SRL_32_bit(
    input [31:0] a,
    input shift,
    output [31:0] out
);
    assign out = a >> shift;
endmodule


module MUX2I_4(
    input wire [3:0]a,
    input wire [3:0]b,
    input wire sel,
    output wire [3:0] out
);

    wire [3:0]options[1:0];
    assign options[0] = a;
    assign options[1] = b;
    assign out = options[sel];
endmodule


module MUX2I(
    input wire [31:0]a,
    input wire [31:0]b,
    input wire sel,
    output wire [31:0] out
);

    wire [31:0]options[1:0];
    assign options[0] = a;
    assign options[1] = b;
    assign out = options[sel];
endmodule

module MUX3I(
    input wire [31:0]a,
    input wire [31:0]b,
    input wire [31:0]c,
    input wire [1:0]sel,
    output wire [31:0] out
);

    wire [31:0]options[2:0];
    assign options[0] = a;
    assign options[1] = b;
    assign options[2] = c;
    assign out = options[sel];
endmodule

module CMP(
    input wire [31:0]a,
    output wire eqz,
    output wire gz,
    output wire lz
);
    assign eqz = (a == 0);
    assign gz = (a > 0);
    assign lz = (a < 0);
endmodule


module SIGN_MOD(input [15:0]imm,output [31:0]ex_imm);
    assign ex_imm = {{16{imm[15]}},imm};
endmodule
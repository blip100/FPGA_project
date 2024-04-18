`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04.10.2023 15:34:58
// Design Name: 
// Module Name: alu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module ALU_32_bit(input [31:0]a,input [31:0]b,input [3:0]select ,output [31:0]alu_out);
wire [31:0]out[8:0];
wire ONE,ZERO;
assign ONE = 1;
assign ZERO = 0;
adder_32_bit 		alu1(a,b,ZERO,out[0]);
xor_32_bit 		alu2(a,b,out[1]);
subtracter_32_bit 	alu3(a,b,out[2]);
not_32_bit 		alu4(a,out[3]);
or_32_bit 		alu5(a,b,out[4]);
and_32_bit 		alu6(a,b,out[5]);
SLA_32_bit 		alu7(a,b[0],out[6]);
SRA_32_bit 		alu8(a,b[0],out[7]);
SRL_32_bit 		alu9(a,b[0],out[8]);

assign alu_out = out[select];


endmodule




module gen_slow_clk(slow_clk, fast_clk);
	input fast_clk;
	reg [26:0] counter;
	output wire slow_clk;
	 
	always@(posedge fast_clk)
	begin
	    counter <= counter + 1;
	end
    initial
    begin
        counter = 0;
    end
	   
	assign slow_clk = counter[20];
    // always @(posedge slow_clk) $display("ALU slow_clk");

endmodule



//module top_module(input fast_clk,input man_clk,input reset,input out_sel,input [7:0]data,output [15:0]out,output slow_clk);
//reg [31:0]a,b;
//wire [31:0] alu_out;
//reg [3:0] sel;
//reg [3:0] counter;





//ALU_32_bit alu0(a,b,sel,alu_out);
//gen_slow_clk clk0(slow_clk,fast_clk);

////assign out = alu_out[15:0];

//wire [15:0]out_arr[1:0];
//assign out_arr[0] = alu_out[15:0];
//assign out_arr[1] = alu_out[31:16];
//assign out = out_arr[out_sel];


//initial
//begin
//    counter = 0;
//    a=0;
//    b=0;
//    sel=0;
//end

//always @ (posedge slow_clk)
//begin
//    if(man_clk ==1)
//    begin 
//        case(counter)
//             0 : a[7:0] <= data;
//             1 : a[15:8] <= data;
//             2 : a[23:16] <= data;
//             3 : a[31:24] <= data;
//             4 : b[7:0] <= data;
//             5 : b[15:8] <= data;
//             6 : b[23:16] <= data;
//             7 : b[31:24] <= data;
//             8 : sel <= data[3:0];
//        endcase
//        if(counter < 8) counter <= counter + 1;
//        else counter <= 0; 
//    end 
//    if (reset == 1) 
//    begin
//        counter <= 0;
//        a <= 0;
//        b <= 0;
//        sel <= 0;
//    end
//    $display("counter = %d",counter);
//    $display("a = %d",a);
//    $display("b = %d",b);
//end

//endmodule


module reg_bank(input [3:0]read1,input [3:0]read2,input [3:0]write_addr,input [31:0]write_data,input write_clk,output [31:0]out1,output [31:0]out2);
reg [31:0]regbank[15:0];
assign out1 = regbank[read1];
assign out2 = regbank[read2];
initial begin
    regbank[0] = 0;
    regbank[1] = 1;
    regbank[2] = 2;
    regbank[3] = 3;
    regbank[4] = 4;
    regbank[5] = 5;
    regbank[6] = 6;
    regbank[7] = 7;
    regbank[8] = 8;
    regbank[9] = 9;
    regbank[10] = 10;
    regbank[11] = 11;
    regbank[12] = 12;
    regbank[13] = 13;
    regbank[14] = 14;
    regbank[15] = 15;
end
always @(posedge write_clk)
begin
regbank[write_addr] = write_data;
end
endmodule


// module reg_operation(input [3:0]i1,input [3:0]i2,input [3:0]i3,input fast_clk,input man_clk,output [15:0]out,output slow_clk);
// wire slow_clk;
// gen_slow_clk clk0(slow_clk,fast_clk);
// reg [1:0]counter ;
// reg [3:0]rs,rt,rd,opcode;
// reg [31:0] operand1,operand2;
// reg write_clk;
// wire [31:0] regout1,regout2,alu_out;
// assign out = alu_out[15:0];
// reg_bank bank0(rs,rt,rd,alu_out,write_clk,regout1,regout2);
// ALU_32_bit alu0(operand1,operand2,opcode,alu_out);


// initial begin
//     counter = 0;
//     rs = 0;
//     rt = 0;
//     rd = 0;
//     opcode = 0;
//     write_clk = 0;
//     operand1 = 0;
//     operand2 = 0;
// end

// always@(posedge slow_clk)
// begin
//     if(man_clk==1)
//     begin
//         case(counter)
//              0: begin
//                 rt <= i3;
//                 rs <=i2;
//                 opcode <=i1;
//              end
//              1: begin
//                 operand1 <= regout1;
//                 operand2 <= regout2;
//                 rd <= i3;
//              end
//              2: begin
//                 write_clk = 1;
//              end
//              3: begin
//                 write_clk = 0;
//              end
//         endcase
//         counter <= counter +1;
//     end
// end
// endmodule



module control_path(
    input wire control_clk,
    input wire [5:0]opcode,
    input wire eqz,
    input wire gz,
    input wire lz,
    output reg reg_dst_sel,
    output reg [1:0]alu_sel1,
    output reg alu_sel2,
    output reg [3:0] alu_op,
    output reg [1:0]pc_sel,
    output reg sp_update_sel,
    output reg sp_load_sel,
    output reg sp_write_sel,
    output reg mem_addr_sel,
    output reg mem_data_sel,
    output reg mem_write_signal,
    output reg reg_write_signal,
    output reg sp_write_signal,
    output reg reg_write_sel
);
    always @ (posedge control_clk)
    begin 
        $display("opcode : %d",opcode);
        case(opcode)
            0 : begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 0;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            1 : begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 2;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            2: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 5;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            3: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 4;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            4: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 1;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            6'b000101: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 4'b0110;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            6'b000110: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 4'b0111;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b000111: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 'b1000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b001000: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 'b0011;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b001001: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 1;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 1;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 1;
                sp_write_sel = 0;
            end
            'b001010: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b001011: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 1;
                mem_addr_sel = 1;
                mem_data_sel = 1;
                sp_load_sel = 1;
                sp_update_sel = 1;
                sp_write_signal = 1;
                sp_write_sel = 0;
            end
            'b100000: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b100001: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0010;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b100010: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0101;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b100011: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0100;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b100100: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0001;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b100101: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0110;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b100110: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0111;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b100111: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b1000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b101000: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 1;
                reg_dst_sel = 1;
                reg_write_sel = 1;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b101001: begin   ///////////////////////////// LDSP
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 1;
                sp_write_sel = 1;
            end
            'b101010: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 1;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b101011: begin  ///////////////////////////// STSP
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 1;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b101100: begin
                alu_sel1 = 1;
                alu_sel2 = 1;
                alu_op = 'b0000;
                pc_sel = 1;
                mem_write_signal = 0;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b101101: begin
                alu_sel1 = 1;
                alu_sel2 = 1;
                alu_op = 'b0000;
                if(lz==1) pc_sel = 1;
                else pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b101110: begin
                alu_sel1 = 1;
                alu_sel2 = 1;
                alu_op = 'b0000;
                if(gz==1)pc_sel=1;
                else pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b101111: begin
                alu_sel1 = 1;
                alu_sel2 = 1;
                alu_op = 'b0000;
                if(eqz==1)pc_sel = 1;
                else pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 1;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0;
            end
            'b110000: begin
                alu_sel1 = 0;
                alu_sel2 = 1;
                alu_op = 'b0000;
                pc_sel = 1;
                mem_write_signal = 1;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 1;
                mem_data_sel = 0;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 1;
                sp_write_sel = 0;
            end
            'b110001: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 'b0000;
                pc_sel = 2'b10;
                mem_write_signal = 0;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 1;
                mem_data_sel = 0;
                sp_load_sel = 1;
                sp_update_sel = 1;
                sp_write_signal = 1;
                sp_write_sel = 0;
            end
            'b110010: begin
                $finish;
            end
            'b110011: begin
                alu_sel1 = 0;
                alu_sel2 = 0;
                alu_op = 'b0000;
                pc_sel = 0;
                mem_write_signal = 0;
                reg_write_signal = 0;
                reg_dst_sel = 0;
                reg_write_sel = 0;
                mem_addr_sel = 0;
                mem_data_sel = 0;
                sp_load_sel = 0;
                sp_update_sel = 0;
                sp_write_signal = 0;
                sp_write_sel = 0; 
            end
        
        endcase
    end  
endmodule

module datapath(
    output wire [5:0] opcode,
    output wire eqz,
    output wire gz,
    output wire lz,
    input wire reg_dst_sel,
    input wire [1:0]alu_sel1,
    input wire alu_sel2,
    input wire [3:0] alu_op,
    input wire [1:0]pc_sel,
    input wire sp_update_sel,
    input wire sp_load_sel,
    input wire sp_write_sel,
    input wire mem_addr_sel,
    input wire mem_data_sel,
    input wire mem_write_signal,
    input wire reg_write_signal,
    input wire sp_write_signal,
    input wire reg_write_sel,
    input wire write_clk
);

    reg [31:0]pc;
    reg [31:0]sp;
    wire [31:0]instr;

    initial begin
        pc = 0;
        sp = 1024;
    end

    assign opcode = instr[31:26];
    INSTR_MEMORY instr_mem0(pc,instr);
    
    wire [3:0]rs;
    wire [3:0]rt;
    wire [3:0]rd;
    wire [15:0]imm;
    
    assign rs = instr[25:22];
    assign rt = instr[21:18];
    assign rd = instr[17:14];
    assign imm = instr[15:0];


    wire [3:0] reg_dst;
    wire [31:0] regout1,regout2,reg_write_data;
    MUX2I_4 reg_dst_mux(rd,rt,reg_dst_sel,reg_dst);
    reg_bank reg_bank0(rs,rt,reg_dst,reg_write_data,reg_bank_signal,regout1,regout2);

    wire [31:0]ex_imm;
    SIGN_MOD sign_mod0(imm,ex_imm);

    wire [31:0] alu_in1,alu_in2;
    MUX3I alu_mux1(regout1,pc,sp,alu_sel1,alu_in1);
    MUX2I alu_mux2(regout2,ex_imm,alu_sel2,alu_in2);

    wire [31:0]alu_out;
    ALU_32_bit alu0(alu_in1,alu_in2,alu_op,alu_out);

    wire [31:0] pc_plus_4,new_pc;
    adder_32_bit npc(pc,4,1'b0,pc_plus_4);
    MUX3I PC_update(pc_plus_4,alu_out,mem_out,pc_sel,new_pc);

    wire [31:0] sp_m1_out,sp_m2_out,sp_a1_out,sp_a2_out;
    wire [31:0]temp1 = -4;
    wire [31:0]temp2 = 4;
    MUX2I sp_mux_1(temp1,temp2,sp_update_sel,sp_m1_out);
    adder_32_bit sp_adder_1(sp,sp_m1_out,1'b0,sp_a1_out);

    wire [31:0]temp3 = -4;
    wire [31:0]temp4 = 0;
    MUX2I sp_mux_2(temp3,temp4,sp_load_sel,sp_m2_out);
    adder_32_bit sp_adder_2(sp,sp_m2_out,1'b0,sp_a2_out);

    wire [31:0] new_sp;
    MUX2I sp_mux_3(sp_a1_out,mem_out,sp_write_sel,new_sp);

    wire [31:0] mem_addr,mem_out;
    wire [31:0] data_in;
    MUX2I mem_mux_1(alu_out,sp_a2_out,mem_addr_sel,mem_addr);
    MUX2I mem_mux_2(pc_plus_4,regout2,mem_data_sel,data_in);

    
    MEMORY mem0(mem_addr,data_in,mem_bank_signal,mem_out);
    
    MUX2I reg_mux_w(alu_out,mem_out,reg_write_sel,reg_write_data);

    CMP b_flag(regout1,eqz,gz,lz);
    

    reg reg_bank_signal,mem_bank_signal;

    always @(posedge write_clk)
    begin
        // $display("Instruction : %b",instr[31:16]);
        // $display("Instruction : %b",instr[15:0]);
        // $display("Opcode : %b",instr[31:26]);
        // $display("rs : %b",instr[25:22]);
        // $display("rt : %b",instr[21:18]);
        $display("PC : %d",pc);
        $display("SP : %d",sp);
        // $display("regout1 : %d",regout1);
        // $display("regout2 : %d",regout2);
        // $display("reg_write_data : %d",reg_write_data);
        // $display("alu_out : %d",alu_out);
        // $display("mem_addr : %d",mem_addr);
        // $display("mem_out : %d",mem_out);
        // $display("reg_bank_signal : %d",reg_bank_signal);
        // $display("mem_bank_signal : %d",mem_bank_signal);
        // $display("sp_write_signal : %d",sp_write_signal);
        // $display("reg_write_signal : %d",reg_write_signal);
        // $display("pc_sel : %d",pc_sel);
        
        if(reg_write_signal)
        begin
            reg_bank_signal = 1;
        end
        if(mem_write_signal)
        begin
            mem_bank_signal = 1;
        end
        if(sp_write_signal)
        begin
            sp <= new_sp;
        end
        pc <= new_pc;
    end

    always @(negedge write_clk)
    begin  
        reg_bank_signal = 0;
        mem_bank_signal = 0;
    end

endmodule

module MEMORY(input [31:0]addr, input [31:0]data_in,input write_clk, output [31:0]data_out );
    reg [7:0]mem[0:1023];
    assign data_out = {mem[addr][7:0], mem[addr+1][7:0], mem[addr+2][7:0], mem[addr+3][7:0]};
    always @(posedge write_clk)
    begin
        // $display("Memory Write Data in : %d",data_in);
        // $display("Memory Write Address : %d",addr);
        // $display("Memory Write Data out : %d",data_out);
        mem[addr] = data_in[31:24];
        mem[addr+1] = data_in[23:16];
        mem[addr+2] = data_in[15:8];
        mem[addr+3] = data_in[7:0];
    end
endmodule



module INSTR_MEMORY(input [31:0]addr,output [31:0]instr);
    reg [7:0]mem[0:1023];
    initial begin
        $readmemb("instr_mem.txt",mem);
    end
    assign instr = {mem[addr][7:0], mem[addr+1][7:0], mem[addr+2][7:0], mem[addr+3][7:0]};
endmodule



module top_module(input fast_clk,input man_clk);
    wire slow_clk;
    gen_slow_clk G0(slow_clk,fast_clk);
    reg counter;

    reg control_clk,write_clk;

    initial begin
        counter = 0;
        write_clk = 0;
        control_clk = 0;
    end



    wire [5:0] opcode;
    wire eqz;
    wire gz;
    wire lz;
    wire reg_dst_sel;
    wire [1:0]alu_sel1;
    wire alu_sel2;
    wire [3:0] alu_op;
    wire [1:0]pc_sel;
    wire sp_update_sel;
    wire sp_load_sel;
    wire sp_write_sel;
    wire mem_addr_sel;
    wire mem_data_sel;
    wire mem_write_signal;
    wire reg_write_signal;
    wire sp_write_signal;
    wire reg_write_sel;


    datapath D0(
        opcode[5:0],
        eqz,
        gz,
        lz,
        reg_dst_sel,
        alu_sel1[1:0],
        alu_sel2,
        alu_op[3:0],
        pc_sel[1:0],
        sp_update_sel,
        sp_load_sel,
        sp_write_sel,
        mem_addr_sel,
        mem_data_sel,
        mem_write_signal,
        reg_write_signal,
        sp_write_signal,
        reg_write_sel,
        write_clk
    );


    control_path C0(
        control_clk,
        opcode[5:0],
        eqz,
        gz,
        lz,
        reg_dst_sel,
        alu_sel1[1:0],
        alu_sel2,
        alu_op[3:0],
        pc_sel[1:0],
        sp_update_sel,
        sp_load_sel,
        sp_write_sel,
        mem_addr_sel,
        mem_data_sel,
        mem_write_signal,
        reg_write_signal,
        sp_write_signal,
        reg_write_sel
    );

    always @(posedge slow_clk)
    begin
        $display("\n**********State = %d**********",counter);
        if(man_clk == 1)
        begin
            if(counter==0)
            begin
                control_clk = 1;
                write_clk = 0;
            end
            if(counter==1)
            begin 
                control_clk = 0;
                write_clk = 1;
            end
            counter <= ~counter;
        end
    end
endmodule

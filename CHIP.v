`include "ALU.v"

    `define R_TYPE 7'b0110011 // add, sub, and, xor, mul
    `define I_TYPE  7'b0010011 // addi, slli, slti, srai
    `define I_L_TYPE  7'b0000011 //lw
    `define S_TYPE  7'b0100011 //sw
    `define I_JALR  7'b1100111 //jalr
    `define UJ_TYPE  7'b1101111 //jal
    `define U_TYPE  7'b0010111 //auipc
    `define SB_TYPE  7'b1100011 //bge, blt, bnq, beq

    `define TO_PC_PLUS_4 2'd0
    `define  TO_ALU 2'd1
    `define TO_MEM 2'd2
    `define  TO_PC_PLUS_IMM 2'd3
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//
module CHIP #(                                                                                  //
    parameter BIT_W = 32                                                                        //
)(                                                                                              //
    // clock                                                                                    //
        input               i_clk,                                                              //
        input               i_rst_n,                                                            //
    // instruction memory                                                                       //
        input  [BIT_W-1:0]  i_IMEM_data,                                                        //
        output [BIT_W-1:0]  o_IMEM_addr,                                                        //
        output              o_IMEM_cen,                                                         //
    // data memory                                                                              //
        input               i_DMEM_stall,                                                       //
        input  [BIT_W-1:0]  i_DMEM_rdata,                                                       //
        output              o_DMEM_cen,                                                         //
        output              o_DMEM_wen,                                                         //
        output [BIT_W-1:0]  o_DMEM_addr,                                                        //
        output [BIT_W-1:0]  o_DMEM_wdata                                                        //
);                                                                                              //
//----------------------------- DO NOT MODIFY THE I/O INTERFACE!! ------------------------------//

// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Parameters
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    // TODO: any declaration
    //instruction type stuff
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Wires and Registers
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    // TODO: any declaration
      reg [BIT_W-1:0] PC, next_PC;
        wire mem_cen, mem_wen;
        wire [BIT_W-1:0] mem_addr, mem_wdata, mem_rdata;
        wire mem_stall;
        reg [BIT_W-1:0] wdata;
        reg stall=1,stall_next;
   // ------------------------------------------------------------------------------------------------------------------------------------------------------
// Continuous Assignment
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    //instruction
    wire [24:0] immediate;
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire [4:0] rs1,rs2,rd;
    wire [31:0] rdata1,rdata2;
    wire [BIT_W-1:0] PC_4;
    assign opcode = i_IMEM_data[6:0];
    assign funct3 = i_IMEM_data[14:12];
    assign funct7 = i_IMEM_data[31:25];
    assign immediate = i_IMEM_data[31:7];
    assign PC_4 = PC+4;
    assign rs1=i_IMEM_data[19:15];
    assign rs2=i_IMEM_data[24:20];
    assign rd=i_IMEM_data[11:7];
    //ALU
    wire [2:0]alu_ctrl ;
    wire i_valid;
    wire [31:0]ALUresult;
    wire o_done;
    wire alu_zero;
    wire [31:0]ALUsrc;
    wire reverse;
    //Control
    wire MemRead;
    wire Branch;
    wire MemWrite;
    wire RegWrite;
    wire ALUSrc;
    wire [1:0] MemtoReg;
    wire [6:0]ALUop;
    // TODO: any wire assignment
    //IMM
    wire [31:0]exteimm;
  
    assign o_DMEM_addr= ALUresult;
    assign o_DMEM_wdata=rdata2;
    assign o_IMEM_addr=PC;
    reg flag=0;
   
     reg cen,cen_next;
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Submoddules
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    IMMGEN immgen(
        .instrm(immediate),
        .funct3(funct3),
        .funct7(funct7),
        .opcode(opcode),
        .exteimm(exteimm)//instrm, opcode, exteimm
    );
      
    CONTROL control(
        .opcode(opcode),
        .Branch(Branch),
        .i_DMEM_stall(i_DMEM_stall),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .RegWrite(RegWrite),
        .ALUSrc(ALUSrc),
        .MemtoReg(MemtoReg),
        .ALUop(ALUop)

    );
    // TODO: Reg_file wire connection
    Reg_file reg0(               
        .i_clk  (i_clk),             
        .i_rst_n(i_rst_n),         
        .wen    (RegWrite),          
        .rs1    (rs1),                
        .rs2    (rs2),                
        .rd     (rd),                 
        .wdata  (wdata),             
        .rdata1 (rdata1),           
        .rdata2 (rdata2)
    );

    ALUcontrol alucontrol(
        .ALUop(ALUop),
        .i_clk(i_clk),
        .funct3(funct3),
        .funct7(funct7),
        .alu_ctrl(alu_ctrl),
        .i_valid(i_valid),
        .reverse(reverse),
        .o_done(o_done)
    );
    
    ALUfull alu(
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_valid(i_valid),
        .cen_next(cen_next),
        .i_DMEM_stall(i_DMEM_stall),
        .reverse(reverse),
        .alu_ctrl(alu_ctrl),
        .opcode(opcode),
        .funct7(funct7),
        .input1(rdata1),
        .input2(ALUsrc),
        .ALUresult(ALUresult),
        .alu_zero(alu_zero),
        .o_done(o_done),
        .od_next(od_next)

    );
// ------------------------------------------------------------------------------------------------------------------------------------------------------
// Always Blocks
// ------------------------------------------------------------------------------------------------------------------------------------------------------
    assign o_IMEM_cen=stall;
    always @(*)begin
     if(opcode!=0&&flag==0)begin stall_next=1; flag=1;end 
     else if(flag==1) begin stall_next=0; flag=flag; end
     else begin stall_next=1;flag=0; end
    
     if(o_done==1) begin stall_next=1; flag=0; end
     else begin stall_next=stall_next; flag=flag; end;
      
    end
    always @(posedge i_clk or negedge i_rst_n) begin
         if (!i_rst_n) begin
           
       end
      else 
         begin
             stall <= stall_next;     
         end
    end
        assign ALUsrc=(ALUSrc)?exteimm:rdata2;
    reg [BIT_W-1:0]wtemp1,wtemp1_next;

    // Todo: any combinational/sequential circuit
 always @(*) begin

        if(od_next)begin
            wtemp1_next=i_DMEM_rdata;
             
            end

        else    wtemp1_next=0;
        
     end
    always @(posedge i_clk or negedge i_rst_n) begin
         if (!i_rst_n) begin
    
       end
      else 
         begin
            if(od_next)
             wtemp1 <= wtemp1_next;     
            else
            wtemp1 <=wtemp1;
         end
    end






    always @(*)begin
      case(MemtoReg)
        `TO_PC_PLUS_4: wdata = PC_4;
        `TO_ALU: wdata = ALUresult;
        `TO_MEM: begin if(od_next)wdata =i_DMEM_rdata;else wdata=wtemp1; end
        `TO_PC_PLUS_IMM: wdata = PC + exteimm;
        
      endcase
    end
    always @(*)begin
        if(o_done)begin
            case(opcode)
            `R_TYPE:next_PC=PC_4;
            `I_TYPE:next_PC=PC_4;
            `I_L_TYPE:next_PC=PC_4;
            `S_TYPE:next_PC=PC_4;
            `UJ_TYPE:next_PC=PC+(exteimm<<1);
            `I_JALR:next_PC=ALUresult;
            `U_TYPE:next_PC=PC_4;
            `SB_TYPE:next_PC=(Branch&&alu_zero)?(PC+(exteimm<<1)):PC_4;
            default: next_PC = PC_4;
            endcase
        end
        else next_PC=PC;
    end
   
    assign o_DMEM_cen = cen;
    assign o_DMEM_wen= (MemWrite&&cen);
    parameter S1 =0;
    parameter S2 =1;
    reg stat1,stat1_next;

    always @(*) begin
        if(stat1)begin
            if(cen==1)begin 
                cen_next=0;
                stat1_next=S1;
            end
            else    begin cen_next=(i_DMEM_stall==1)?0:(MemWrite | MemRead);
                    stat1_next = S1;
        end
        end
        else begin
            cen_next=0;
            if(o_done)stat1_next=S2;
            else stat1_next=S1;
        end
     end
    always @(posedge i_clk or negedge i_rst_n) begin
         if (!i_rst_n) begin
             cen <= 0;
             stat1<=0;
       end
      else 
         begin
             cen <= cen_next;     
             stat1<=stat1_next;    
         end
    end

 
    assign i_DMEM_wdata=o_DMEM_wdata;


    
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            PC <= 32'h00010000; // Do not modify this value!!!
        end
        else begin
            if(o_done) PC <= next_PC;
        end
    end
endmodule

module Reg_file(i_clk, i_rst_n, wen, rs1, rs2, rd, wdata, rdata1, rdata2);
   
    parameter BITS = 32;
    parameter word_depth = 32;
    parameter addr_width = 5; // 2^addr_width >= word_depth
    
    input i_clk, i_rst_n, wen; // wen: 0:read | 1:write
    input [BITS-1:0] wdata;
    input [addr_width-1:0] rs1, rs2, rd;

    output [BITS-1:0] rdata1, rdata2;

    reg [BITS-1:0] mem [0:word_depth-1];
    reg [BITS-1:0] mem_nxt [0:word_depth-1];

    integer i;

    assign rdata1 = mem[rs1];
    assign rdata2 = mem[rs2];

    always @(*) begin
        for (i=0; i<word_depth; i=i+1)
            mem_nxt[i] = (wen && (rd == i)) ? wdata : mem[i];
    end

    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1) begin
                case(i)
                    32'd2: mem[i] <= 32'hbffffff0;
                    32'd3: mem[i] <= 32'h10008000;
                    default: mem[i] <= 32'h0;
                endcase
            end
        end
        else begin
            mem[0] <= 0;
            for (i=1; i<word_depth; i=i+1)
                mem[i] <= mem_nxt[i];
        end       
    end
endmodule

module CONTROL (opcode, Branch, MemRead, MemtoReg, ALUop, MemWrite, ALUSrc, RegWrite,i_DMEM_stall);
        //CONTROL Branch, MemRead, MemWrite, RegWrite
        parameter TRUE = 1'b1;
        parameter FALSE = 1'b0;
        // MemtoReg
        // parameter TO_PC_PLUS_4 = 2'b0;
        // parameter TO_ALU = 2'd1;
        // parameter TO_MEM = 2'd2;
        // parameter TO_PC_PLUS_IMM = 2'd3;
        // ALU_source
        parameter RS2 = 1'b0;
        parameter IMM = 1'b1;
        input i_DMEM_stall;
        input [6:0] opcode;
        output reg Branch, MemRead, MemWrite, RegWrite, ALUSrc;
        output reg [1:0] MemtoReg;
        output reg [6:0] ALUop;
        always @(*) begin
                case(opcode)
                    `R_TYPE:begin
                        Branch = FALSE;
                        MemRead = FALSE;
                        MemtoReg = `TO_ALU;
                        ALUop = opcode;
                        MemWrite = FALSE;
                        ALUSrc = RS2;
                        RegWrite = TRUE;
                    end
                    `I_TYPE:begin
                        Branch = FALSE;
                        MemRead = FALSE;
                        MemtoReg = `TO_ALU;
                        ALUop = opcode;
                        MemWrite = FALSE;
                        ALUSrc = IMM;
                        RegWrite = TRUE;
                    end
                    `I_L_TYPE:begin
                        Branch = FALSE;
                        MemRead=TRUE;
                        MemtoReg = `TO_MEM;
                        ALUop = opcode;
                        MemWrite = FALSE;
                        ALUSrc = IMM;
                        RegWrite = TRUE;
                    end
                    `S_TYPE:begin
                        Branch = FALSE;
                        MemRead = FALSE;
                        MemtoReg = `TO_MEM;
                        ALUop = opcode;
                        MemWrite = TRUE;
                        ALUSrc = IMM;
                        RegWrite = FALSE;
                    end
                    `I_JALR:begin
                        Branch = FALSE;
                        MemRead = FALSE;
                        MemtoReg = `TO_PC_PLUS_4;
                        ALUop = opcode;
                        MemWrite = FALSE;
                        ALUSrc = IMM;
                        RegWrite = TRUE;
                    end
                    `UJ_TYPE:begin
                        Branch = FALSE;
                        MemRead = FALSE;
                        MemtoReg = `TO_PC_PLUS_4;
                        ALUop = opcode;
                        MemWrite = FALSE;
                        ALUSrc = IMM;
                        RegWrite = TRUE;
                    end
                    `U_TYPE:begin
                        Branch = FALSE;
                        MemRead = FALSE;
                        MemtoReg = `TO_PC_PLUS_IMM;
                        ALUop = opcode;
                        MemWrite = FALSE;
                        ALUSrc = RS2;
                        RegWrite = TRUE;
                    end
                    `SB_TYPE:begin
                        Branch = TRUE;
                        MemRead = FALSE;
                        MemtoReg = `TO_ALU;
                        ALUop = opcode;
                        MemWrite = FALSE;
                        ALUSrc = RS2;
                        RegWrite = FALSE;
                    end
                    default:begin
                        Branch = 0;
                        MemRead = 0;
                        MemtoReg = 0;
                        ALUop = 0;
                        MemWrite = 0;
                        ALUSrc = 0;
                        RegWrite = 0;
                    end
                endcase
        end
endmodule

module IMMGEN(
        input [24:0] instrm,
        input [6:0] opcode,
        input [2:0] funct3,
        input [6:0] funct7,
        output [31:0] exteimm
        );
        reg [31:0] temp;
        assign exteimm = temp;
        always @(*) begin
                case(opcode)
                `R_TYPE: temp = 0;
                `I_TYPE:begin
                     if (funct3==3'b101)
                        temp={{27{instrm[24]}}, instrm[17:13]};
                    else
                     temp = {{20{instrm[24]}}, instrm[24:13]};
                end
                `I_L_TYPE: temp = {{20{instrm[24]}}, instrm[24:13]};
                `S_TYPE: temp = {{20{instrm[24]}}, instrm[24:18],instrm[4:0]};
                `I_JALR: temp = {{20{instrm[24]}}, instrm[24:13]};
                `UJ_TYPE: temp = {{13{instrm[24]}},instrm[12:5],instrm[13],instrm[23:14]};
                `U_TYPE: temp = {instrm[24:5], 12'b0};
                `SB_TYPE: temp = {{21{instrm[24]}},instrm[0],instrm[23:18],instrm[4:1]};
                default: temp = 0;
                endcase
        end
endmodule

module ALUcontrol(
    // TODO: port declaration
    input [6:0]ALUop,
    input [2:0]funct3,
    input [6:0]funct7,
    input i_clk,
    input o_done,
    output reg [2:0] alu_ctrl,
    output reg reverse,
    output reg i_valid
    );
    parameter ADD = 3'd0;
    parameter SUB = 3'd1;
    parameter AND = 3'd2;
    parameter XOR = 3'd3;
    parameter SLT = 3'd4;
    parameter SRA = 3'd5;
    parameter MUL = 3'd6;
    parameter SLL = 3'd7;
    parameter valid=2'd0;
    parameter invalid1=2'd1;
    parameter invalid2=2'd2;
    reg [1:0] sta,stanext;
    always @(*) begin
        case(ALUop)
            `R_TYPE:begin
                case(funct7)
                7'b0000000:begin
                    case(funct3)
                    3'b000:begin
                        alu_ctrl=ADD;reverse=0;
                    end
                    3'b001:begin
                        alu_ctrl=SLL;reverse=0;
                    end
                    3'b010:begin
                        alu_ctrl=SLT;reverse=0;
                    end
                    3'b111:begin
                        alu_ctrl=AND;reverse=0;
                    end
                    3'b100:begin
                        alu_ctrl=XOR;reverse=0;
                    end
                    default:begin
                        alu_ctrl=XOR;reverse=0;
                    end
                    endcase
                end
                7'b0000001:begin
                        alu_ctrl=MUL;reverse=0;
                end
                7'b0100000:begin
                    case(funct3)
                    3'b000:begin
                        alu_ctrl=SUB;reverse=0;
                    end
                    default:begin
                        alu_ctrl=XOR;reverse=0;
                    end
                    endcase
                end
                default:begin
                        alu_ctrl=XOR;reverse=0;
                    end
                endcase
            end
            `I_TYPE:begin
                case(funct3)
                3'b000:begin
                        alu_ctrl=ADD;reverse=0;
                    end
                3'b001:begin
                        alu_ctrl=SLL;reverse=0;
                    end    
                3'b010:begin
                        alu_ctrl=SLT;reverse=0;
                        
                    end 
                3'b101:begin
                        alu_ctrl=SRA;reverse=0;
                    end   
                    default:begin
                        alu_ctrl=XOR;reverse=0;
                    end        
                endcase
            end
            `SB_TYPE:begin
                case(funct3)
                3'b000:begin
                        alu_ctrl=SUB;reverse=0; //BEQ
                    end
                3'b001:begin
                        alu_ctrl=SUB;reverse=1; //BNE
                    end    
                3'b100:begin
                        alu_ctrl=SLT;reverse=1; //BLT
                        
                    end 
                3'b101:begin
                        alu_ctrl=SLT;reverse=0; //BGE
                    end  
                    default:begin
                        alu_ctrl=XOR;reverse=0;
                    end         
                endcase
            end
            
            `U_TYPE:begin
    
                        alu_ctrl=ADD;reverse=0; //AUPIC
          
            end

        default begin
            alu_ctrl=ADD;
        
            reverse=0;
        end
        endcase
 
    end

     always @(*) begin
      
            case(sta)
                invalid1  : 
                begin
                i_valid=0;
                if (ALUop==`R_TYPE&&funct7==7'b0000001) 
                stanext = valid;
                else stanext = invalid1;
                end
                valid:begin i_valid=1; stanext=invalid2; end
                invalid2:begin
                i_valid=0;
                if (o_done) 
                stanext = invalid1;
                else stanext = invalid2;
                end
                default : begin stanext = invalid1; 
                    i_valid=0;
                end
            endcase
          
        end
    always @(posedge i_clk ) begin
         
         begin
             sta <= stanext;     
         end
    end
    // Todo: HW2
    
endmodule
module ALUfull(
    // TODO: port declaration
    input   i_clk,
    input   i_rst_n,
    input   i_valid,
    input cen_next,
    input   reverse,
    input i_DMEM_stall,
    input [6:0]funct7,
    input   [31:0]  input1,
    input   [31:0]  input2,
    input   [2:0]   alu_ctrl,
    input [6:0] opcode,
    output  [31:0]  ALUresult,
    output  alu_zero,
    output  o_done,
    output reg od_next
    );
     reg [31:0] alu_result;
    wire [63:0] o_data;
    reg [31:0]  inpu1;
    assign ALUresult = alu_result;
    assign alu_zero = (reverse==0)?((alu_result == 0)? 1: 0):((alu_result == 0)? 0: 1);
    reg od,o_don1;
    assign o_done=od;
    parameter ADD = 3'd0;
    parameter SUB = 3'd1;
    parameter AND = 3'd2;
    parameter XOR = 3'd3;
    parameter SLT = 3'd4;
    parameter SRA = 3'd5;
    parameter MUL = 3'd6;
    parameter SLL = 3'd7;
    ALU alu(
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_valid(i_valid),
        .i_inst(alu_ctrl),
        .i_A(input1),
        .i_B(input2),
        .o_don(o_don),
        .o_data(o_data)
    );
    reg   state, next_state;
    always @(*) begin
        o_don1=0;
        if(o_done==0)begin
            if(opcode==`R_TYPE&&funct7==7'b0000001)alu_result=input1;
            else
        case(alu_ctrl)
        ADD:begin 
            alu_result=input1+input2;
            o_don1=1;
        end
        SUB:begin
            alu_result=input1-input2;
        o_don1=1;
        end
        AND:begin
        alu_result=input1&input2;
        o_don1=1;
        end
        XOR:begin
            alu_result=input1^input2;
        o_don1=1;
        end
        SLT:begin
            alu_result=$signed(input1)<$signed(input2)? 1:0;
        o_don1=1;
        end
        SRA:begin
            alu_result=$signed(input1) >>> input2;
        o_don1=1;
        end
        SLL:begin
            alu_result= input1 << input2;
        o_don1=1;
        end
        MUL:alu_result = o_data[31:0];
        endcase
        end
        else if(opcode==`R_TYPE&&funct7==7'b0000001) alu_result=o_data[31:0];
        else alu_result=inpu1;
        if(od==1)od_next=0;
        else if(alu_ctrl==MUL)
        od_next=o_don;
        else od_next=(i_DMEM_stall==1||cen_next==1)?0:(o_don||o_don1);
    end
   always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            od <= 0;

        end
        else 
        begin
            if(alu_ctrl==MUL)
            od<=o_don;
            else
            
            od <= od_next;     
            inpu1<=alu_result;    
        end
    end

    // Todo: HW2
    
endmodule

module Cache#(
        parameter BIT_W = 32,
        parameter ADDR_W = 32
    )(
        input i_clk,
        input i_rst_n,
        // processor interface
            input i_proc_cen,
            input i_proc_wen,
            input [ADDR_W-1:0] i_proc_addr,
            input [BIT_W-1:0]  i_proc_wdata,
            output [BIT_W-1:0] o_proc_rdata,
            output o_proc_stall,
        // memory interface
            output o_mem_cen,
            output o_mem_wen,
            output [ADDR_W-1:0] o_mem_addr,
            output [BIT_W-1:0]  o_mem_wdata,
            input [BIT_W-1:0] i_mem_rdata,
            input i_mem_stall
    );

    //---------------------------------------//
    //          default connection           //
    assign o_mem_cen = i_proc_cen;        //
    assign o_mem_wen = i_proc_wen;        //
    assign o_mem_addr = i_proc_addr;      //
    assign o_mem_wdata = i_proc_wdata;    //
    assign o_proc_rdata = i_mem_rdata;    //
    assign o_proc_stall = i_mem_stall;    //
    //---------------------------------------//

    // Todo: BONUS
endmodule
module ALU #(
    parameter DATA_W = 32
)
(
    input                       i_clk,
    input                       i_rst_n,

    input                       i_valid,
    input [DATA_W - 1 : 0]      i_A,
    input [DATA_W - 1 : 0]      i_B,
    input [         2 : 0]      i_inst,

    output [2*DATA_W - 1 : 0]   o_data,
    output                      o_don
);
// Do not Modify the above part !!!

// Parameters
    // Definition of states
    parameter S_IDLE =4'd10;
    parameter S_ADD = 4'd0;
    parameter S_SUB = 4'd1;
    parameter S_AND = 4'd2;
    parameter S_XOR  = 4'd3;
    parameter S_SLT = 4'd4;
    parameter S_SRA = 4'd5;
    parameter S_MUL = 4'd6;
    parameter S_SLL = 4'd7;
    parameter S_OUT = 4'd9;

// Wires & Regs
    // Todo
    reg  [3:0] state, next_state;
    reg  [4:0] counter, next_counter;
    reg  [2*DATA_W - 1:0] shreg, next_shreg,reg1;
    reg  [DATA_W - 1:0] alu_out;
    reg [DATA_W:0] temp;

// Wire Assignments
    // Todo
    assign o_data = reg1;
    assign o_don = (state == S_OUT) ? 1 : 0;
    

// Always Combination
    // Todo: FSM
        always @(*) begin
            case(state)
                S_IDLE  : 
                begin
                if (i_valid) next_state = i_inst;
                else next_state = S_IDLE;
                end
                S_ADD   : next_state = S_OUT;
                S_SUB   : next_state = S_OUT;
                S_AND   : next_state = S_OUT;
                S_XOR    : next_state = S_OUT;
                S_SLT   : next_state = S_OUT;
                S_SRA   : next_state = S_OUT;
                S_MUL   : next_state = (counter==5'd31) ? S_OUT : S_MUL;
                S_SLL   : next_state = S_OUT;
                S_OUT   : next_state = S_IDLE;
                default : next_state = S_IDLE;
            endcase
        end
    // Todo: Counter
        always @(*) begin
        case(state)
            S_MUL: next_counter = (counter < 6'd32) ? counter + 1'b1 : 0;
            default: next_counter = 0;
        endcase
    end
   
    // Todo: ALU output
        always @(*) begin
            alu_out=0;
            temp=0;
        case(state)
            S_ADD :alu_out= i_A+i_B;
            S_SUB :alu_out= $signed(i_A)-$signed(i_B);
            S_AND :  alu_out= i_A&i_B;
            S_XOR : alu_out= i_A^i_B;
            S_SLT :begin 
            alu_out=$signed(i_A)<$signed(i_B)? 1:0;
            end
            S_SRA :alu_out=$signed(i_A) >>> i_B;
            S_MUL : temp = {1'b0, shreg[63:32]} + (shreg[0] ? i_B : 0);
            S_SLL : alu_out = i_A << i_B;
            default: alu_out = 0;
        endcase
    end
    // Todo: Shift register
        always @(posedge i_clk) begin
        
        case(state)
        
            S_IDLE: begin
                if (i_valid) next_shreg = {32'b0, i_A};
                else next_shreg = 0;
            
            end
           
            S_MUL : begin
               
                next_shreg[30:0] = shreg[31:1];
                next_shreg[63:31] = temp;
              
            end
            
            default: next_shreg = 0;
        endcase
        end
    // Todo: Sequential always block
     always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            state <= S_IDLE;
            counter <= 0;
            shreg <= 0;
  

        end
        else 
        begin
            state <= next_state;
            counter <= next_counter;
            shreg <= next_shreg;
            if(state==S_MUL)reg1<=next_shreg;          
        end
    end

endmodule
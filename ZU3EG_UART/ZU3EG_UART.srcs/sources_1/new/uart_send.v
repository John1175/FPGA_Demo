`timescale 1ns / 1ps
// +FHDR---------------------------------------------------------------------------
// Copyrigth (c) 2024, SIMTECT
// Xingmai Confidential Proprietary
// --------------------------------------------------------------------------------
// FILE NAME		: uart_send.v
// TYPE			: module
// DEPARTMENT		:
// AUTHOR		:
// AUTHOR'S EMAIL	:
// --------------------------------------------------------------------------------
// Release history
// VERSION	DATE		AUTHOR		DESCRIPTION
//
// --------------------------------------------------------------------------------
// KEYWORDS :  uart_send
// --------------------------------------------------------------------------------
// PURPOSE :
//      发送串口数据
// -FHDR---------------------------------------------------------------------------

module uart_send
#(
	parameter CLK_FRE = 50,         //时钟频率，默认时钟频率为50MHz
	parameter DATA_WIDTH = 8,       //有效数据位，缺省为8位
	parameter PARITY_ON = 0,        //校验位，1为有校验位，0为无校验位，缺省为0
	parameter PARITY_TYPE = 0,      //校验类型，1为奇校验，0为偶校验，缺省为偶校验
	parameter BAUD_RATE = 9600      //波特率，缺省为9600
)
(
	input                         i_clk_sys,        //系统时钟
	input                         i_rst_n,          //系统复位
	input  [DATA_WIDTH - 1:0]     i_tx_data,        //需发送的数据
	input                         i_data_valid,     //传输数据有效标志
	output                        o_txd             //UART串行输出
);

localparam STATE_IDLE    = 3'b000;         //空闲状态
localparam STATE_START   = 3'b001;         //开始状态
localparam STATE_DATA    = 3'b011;         //数据发送状态
localparam STATE_PARITY  = 3'b100;         //数据校验计算和发送
localparam STATE_END     = 3'b101;         //结束状态
localparam CYCLE = CLK_FRE * 1000000 / BAUD_RATE;   //波特计数周期

//--------------------------------------------------------------------
// 定义变量
//--------------------------------------------------------------------
//reg                      rxd_r1,rxd_r2,rxd_r3;
reg  [4:0]               flag_rcv_start  = 'b11111;
reg  [2:0]               current_state   = 'd0;     //当前状态
reg  [2:0]               next_state      = 'd0;     //次态
reg                      baud_valid      = 'b0;     //波特计数有效位
reg  [15:0]              baud_cnt        = 'd0;     //波特率计数器
reg                      baud_pulse      = 'b0;     //波特率采样脉冲
reg  [3:0]               tx_cnt          = 'd0;     //发送数据位计数
reg                      txd;
reg  [DATA_WIDTH - 1:0]  tx_data;
reg                      parity_check;

//--------------------------------------------------------------------
// I/O Connections assignments
//--------------------------------------------------------------------
assign o_txd        = txd;

//--------------------------------------------------------------------
// 波特率计数器和波特采样脉冲（位于数据中心）
//--------------------------------------------------------------------
//波特率计数器
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		baud_cnt <= 'd0;
	end
        else if(!baud_valid) begin
		baud_cnt <= 'd0;
	end
        else if(baud_cnt == CYCLE - 1) begin
		baud_cnt <= 'd0;
	end
        else begin
                baud_cnt <= baud_cnt + 'd1;
	end
end

//波特采样脉冲
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		baud_pulse <= 'b0;
	end
	else if(baud_cnt == CYCLE/2-1) begin
                baud_pulse <= 'b1;
	end
	else begin
                baud_pulse <= 'b0;
        end
end

//--------------------------------------------------------------------
// 数据发送状态机
//--------------------------------------------------------------------
//状态机状态变化定义
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		current_state <= STATE_IDLE;
	end
	else if(!baud_valid) begin
		current_state <= STATE_IDLE;
	end
	else if(baud_valid && baud_cnt == 'd0) begin
		current_state <= next_state;
        end
end

//状态机次态定义
always @ (*) begin
	case(current_state)
		STATE_IDLE: begin
			next_state <= STATE_START;
		end
		STATE_START: begin
			next_state <= STATE_DATA;
		end
		STATE_DATA: begin
			if(tx_cnt == DATA_WIDTH) begin
				if(PARITY_ON == 0) begin          //校验位未启用，直接结束
					next_state <= STATE_END;
				end
				else begin
					next_state <= STATE_PARITY;       //校验位开启时进入校验状态
				end
                        end
			else begin
				next_state <= STATE_DATA;
                        end
		end
		STATE_PARITY: begin
			next_state <= STATE_END;
		end
		STATE_END: begin
			next_state <= STATE_IDLE;
		end
		default: begin
			next_state <= STATE_IDLE;
		end
	endcase
end

//状态机输出逻辑
always @ (posedge i_clk_sys) begin
	if(!i_rst_n) begin
		baud_valid    <= 'b0;
		tx_data       <= 'd0;
		txd           <= 'b1;
		tx_cnt        <= 'd0;
		parity_check  <= 'b0;
	end
	else begin
		case(current_state)
			STATE_IDLE: begin
                            //闲置状态下对寄存器进行复位
			    txd          <= 'b1;
			    tx_cnt       <= 'd0;
                            parity_check <= 'b0;
			    if(i_data_valid) begin
				    baud_valid  <= 'b1;
				    tx_data     <= i_tx_data;
			    end
                        end
			STATE_START: begin
				if(baud_pulse) begin
					txd    <= 'b0;
				end
			end
			STATE_DATA: begin
				if(baud_pulse) begin
					tx_cnt       <= tx_cnt + 'd1;                           //数据位计数
					txd          <= tx_data[0];                             //取出最低位发送
					parity_check <= parity_check + tx_data[0];              //校验位做加法验证高电平的奇偶
					tx_data      <= {1'b0,tx_data[DATA_WIDTH-1:1]};         //数据移位
                                end
                        end
			STATE_PARITY: begin
				if(baud_pulse) begin
					if(PARITY_TYPE == 1) begin
						txd  <= parity_check;
					end
					else begin
						txd  <= parity_check + 'b1;
					end
                                end
                        end
			STATE_END: begin
				if(baud_pulse) begin
					txd          <= 1'b1;
					baud_valid   <= 1'b0;
                                end
			end
			default: begin
				txd          <= 'b1;
				tx_cnt       <= 'd0;
				parity_check <= 'b0;
				baud_valid   <= 'b0;
				tx_data      <= 'd0;
			end
               endcase
       end
end

endmodule

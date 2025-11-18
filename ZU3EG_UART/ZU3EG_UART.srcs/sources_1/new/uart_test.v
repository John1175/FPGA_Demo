`timescale 1ns / 1ps
// +FHDR---------------------------------------------------------------------------
// Copyrigth (c) 2025, SIMTECT
// SIMTECT Confidential Proprietary
// --------------------------------------------------------------------------------
// FILE NAME		: uart_test.v
// TYPE			: module
// DEPARTMENT		:
// AUTHOR		:
// AUTHOR'S EMAIL	:
// --------------------------------------------------------------------------------
// Release history
// VERSION	DATE		AUTHOR		DESCRIPTION
//
// --------------------------------------------------------------------------------
// KEYWORDS :  uart_test
// --------------------------------------------------------------------------------
// PURPOSE :
//     1.UART自动发送数据
//     2.接收信号回传
// -FHDR---------------------------------------------------------------------------

module uart_test(
	input					i_clk_sys,			//输入时钟，100MHz
	input					i_rst_n,			//复位信号
	input					i_uart_rxd,			//串口输入
	output					o_uart_txd			//串口输出
);

//串口设置
localparam	FREQ_CLK           = 100;		//定时时钟频率，单位：MHz
localparam	DATA_WIDTH         = 8;			//有效数据位，缺省为8位
localparam	PARITY_ON          = 0;			//校验位，1为有校验位，0为无校验位，缺省为0
localparam	PARITY_TYPE        = 0;			//校验类型，1为奇校验，0为偶校验，缺省为偶校验
localparam	BAUD_RATE          = 115200;	//波特率，缺省为9600

//解析状态机
localparam	IDLE            = 4'd0;
localparam	START           = 4'd1;
localparam	GPGGA_UTC       = 4'd2;
localparam	GPGGA_LAT       = 4'd3;
localparam	GPGGA_LAT_DIR   = 4'd4;
localparam	GPGGA_LON       = 4'd5;
localparam	GPGGA_LON_DIR   = 4'd6;
localparam	GPGGA_ALT       = 4'd7;
localparam	GPGGA_END       = 4'd8;
localparam   GPZDA_DAY       = 4'd9;
localparam   GPZDA_MONTH     = 4'd10;
localparam   GPZDA_YEAR      = 4'd11;
localparam   GPZDA_END       = 4'd12;
localparam   GNHPR_HEADING   = 4'd13;
localparam   GNHPR_END       = 4'd14;

//--------------------------------------------------------------------
// 定义变量
//--------------------------------------------------------------------
reg				rst_n_r1;					//串口1复位
reg				rst_n_r2;					//串口2复位

//串口数据
wire			valid_rx;
wire			valid_tx;
wire [7:0]		data_rx;
wire [7:0]		data_tx;
reg				rs422_bd_data_rxd_r1;
reg				rs422_bd_data_rxd_r2;
reg				rs422_bd_pps_rxd_r1;
reg				rs422_bd_pps_rxd_r2;
reg				rs422_bd_pps_rxd_r3;
wire			ld_parity;
wire			data_valid;




//--------------------------------------------------------------------
// 对输入单bit信号打节拍
//--------------------------------------------------------------------
always @ (posedge i_clk_sys) begin
	rst_n_r1              <= i_rst_n;
	rst_n_r2              <= rst_n_r1;
	rs422_bd_data_rxd_r1  <= i_rs422_bd_data_rxd;
	rs422_bd_data_rxd_r2  <= rs422_bd_data_rxd_r1;
	// rs422_bd_pps_rxd_r1   <= i_rs422_bd_pps_rxd;
	// rs422_bd_pps_rxd_r2   <= rs422_bd_pps_rxd_r1;
	// rs422_bd_pps_rxd_r3   <= rs422_bd_pps_rxd_r2;
end

//--------------------------------------------------------------------
// 串口接收数据
//--------------------------------------------------------------------
// U1:串口接收
uart_recv
#(
	.CLK_FRE        (FREQ_CLK             ),       //时钟频率，单位MHz，缺省为50MHz
	.DATA_WIDTH     (DATA_WIDTH           ),       //有效数据位，缺省为8
	.PARITY_ON      (PARITY_ON            ),       //校验位，1为有校验位；0为无校验位，缺省为0
	.PARITY_TYPE    (PARITY_TYPE          ),       //校验类型，1为奇校验，0为偶校验，缺省为偶校验
	.BAUD_RATE      (BAUD_RATE            )        //波特率，缺省为9600
) u1_uart_recv(
	.i_clk_sys      (i_clk_sys            ),       //系统时钟
	.i_rst_n        (rst_n_r1             ),       //系统复位
	.i_rxd          (rs422_bd_data_rxd_r2 ),       //输入串口位
	.o_rx_data      (data_rx              ),       //解析出的字节数据
	.o_ld_parity    (ld_parity            ),       //校验位检验LED，高电平为校验正确
	.o_data_valid   (valid_rx             )        //数据有效标志
);

//--------------------------------------------------------------------
// 串口发送数据
//--------------------------------------------------------------------
// U2:串口发送
uart_send
#(
	.CLK_FRE        (FREQ_CLK             ),       //时钟频率，单位MHz，缺省为50MHz
	.DATA_WIDTH     (DATA_WIDTH           ),       //有效数据位，缺省为8
	.PARITY_ON      (PARITY_ON            ),       //校验位，1为有校验位；0为无校验位，缺省为0
	.PARITY_TYPE    (PARITY_TYPE          ),       //校验类型，1为奇校验，0为偶校验，缺省为偶校验
	.BAUD_RATE      (BAUD_RATE            )        //波特率，缺省为9600
) u2_uart_send(
	.i_clk_sys      (i_clk_sys            ),       //系统时钟
	.i_rst_n        (rst_n_r2             ),       //系统复位
	.i_tx_data      (data_tx              ),       //需发送的数据
	.i_data_valid   (data_valid            ),       //传输数据有效标志
	.o_txd			(valid_tx             )        //UART串行输出
);

//--------------------------------------------------------------------
// 解析北斗信息
//--------------------------------------------------------------------
//移位输入byte
always @ (posedge i_clk_sys) begin
	if(valid_rx) begin
		data_rx_mem[0]  <= data_rx;
	end
end

always @ (posedge i_clk_sys) begin
	valid_data  <= valid_rx;
end

genvar index_byte;
generate
	for(index_byte=0; index_byte < 14; index_byte=index_byte+1)begin : u_data_shift
		always @ (posedge i_clk_sys) begin
			if(valid_rx) begin
				data_rx_mem[index_byte+1]  <= data_rx_mem[index_byte];
			end
		end
	end
endgenerate

//解析信息
always @ (posedge i_clk_sys) begin
	if(valid_data) begin
		if(data_rx_mem[0] == C_DOLLAR) begin     //判断起始标识符
			valid_gpgga  <= 'b0;
			valid_gpzda  <= 'b0;
			valid_gnhpr  <= 'b0;
			cnt_byte     <= 'd0;
			cnt_dly      <= 'd0;
			state_parse  <= START;
		end
		else begin
			case(state_parse)
				IDLE: begin
					valid_gpgga  <= 'b0;
					valid_gpzda  <= 'b0;
					valid_gnhpr  <= 'b0;
					cnt_byte     <= 'd0;
					cnt_dly      <= 'd0;
					state_parse  <= IDLE;
				end
				START: begin
					if(data_rx_mem[0] == C_COMMA) begin    //,
						if(data_rx_mem[5] == C_G && data_rx_mem[4] == C_P && data_rx_mem[3] == C_G && data_rx_mem[2] == C_G && data_rx_mem[1] == C_A) begin
							state_parse  <= GPGGA_UTC;
						end
						else if(data_rx_mem[5] == C_G && data_rx_mem[4] == C_P && data_rx_mem[3] == C_Z && data_rx_mem[2] == C_D && data_rx_mem[1] == C_A) begin
							state_parse  <= GPZDA_DAY;
						end
						else if(data_rx_mem[5] == C_G && data_rx_mem[4] == C_N && data_rx_mem[3] == C_H && data_rx_mem[2] == C_P && data_rx_mem[1] == C_R) begin
							state_parse  <= GNHPR_HEADING;
						end
						else begin
							state_parse  <= IDLE;
						end
					end
					else begin
						state_parse  <= START;
					end
				end
				GPGGA_UTC: begin
					if(data_rx_mem[0] == C_COMMA) begin
						utc_hour_1_temp    <= data_rx_mem[9] - 8'h30;
						utc_hour_2_temp    <= data_rx_mem[8] - 8'h30;
						utc_min_1_temp     <= data_rx_mem[7] - 8'h30;
						utc_min_2_temp     <= data_rx_mem[6] - 8'h30;
						utc_sec_1_temp     <= data_rx_mem[5] - 8'h30;
						utc_sec_2_temp     <= data_rx_mem[4] - 8'h30;
						utc_sec_dot_1_temp <= data_rx_mem[2] - 8'h30;
						utc_sec_dot_2_temp <= data_rx_mem[1] - 8'h30;
						state_parse        <= GPGGA_LAT;
					end
				end
				GPGGA_LAT: begin
					if(data_rx_mem[0] == C_COMMA) begin
						lat_deg_1_temp     <= data_rx_mem[13] - 8'h30;
						lat_deg_2_temp     <= data_rx_mem[12] - 8'h30;
						lat_min_1_temp     <= data_rx_mem[11] - 8'h30;
						lat_min_2_temp     <= data_rx_mem[10] - 8'h30;
						lat_min_dot_1_temp <= data_rx_mem[8]  - 8'h30;
						lat_min_dot_2_temp <= data_rx_mem[7]  - 8'h30;
						lat_min_dot_3_temp <= data_rx_mem[6]  - 8'h30;
						lat_min_dot_4_temp <= data_rx_mem[5]  - 8'h30;
						lat_min_dot_5_temp <= data_rx_mem[4]  - 8'h30;
						lat_min_dot_6_temp <= data_rx_mem[3]  - 8'h30;
						lat_min_dot_7_temp <= data_rx_mem[2]  - 8'h30;
						lat_min_dot_8_temp <= data_rx_mem[1]  - 8'h30;
						state_parse        <= GPGGA_LAT_DIR;
					end
				end
				GPGGA_LAT_DIR: begin
					if(data_rx_mem[0] == C_COMMA) begin
						if(data_rx_mem[1] == C_N) begin  //北纬，0
							lat_dir_temp    <= 'b0;
						end
						else begin
							lat_dir_temp    <= 'b1;  //南纬，1
						end
						state_parse        <= GPGGA_LON;
					end
				end
				GPGGA_LON: begin
					if(data_rx_mem[0] == C_COMMA) begin
						lon_deg_1_temp     <= data_rx_mem[14] - 8'h30;
						lon_deg_2_temp     <= data_rx_mem[13] - 8'h30;
						lon_deg_3_temp     <= data_rx_mem[12] - 8'h30;
						lon_min_1_temp     <= data_rx_mem[11] - 8'h30;
						lon_min_2_temp     <= data_rx_mem[10] - 8'h30;
						lon_min_dot_1_temp <= data_rx_mem[8]  - 8'h30;
						lon_min_dot_2_temp <= data_rx_mem[7]  - 8'h30;
						lon_min_dot_3_temp <= data_rx_mem[6]  - 8'h30;
						lon_min_dot_4_temp <= data_rx_mem[5]  - 8'h30;
						lon_min_dot_5_temp <= data_rx_mem[4]  - 8'h30;
						lon_min_dot_6_temp <= data_rx_mem[3]  - 8'h30;
						lon_min_dot_7_temp <= data_rx_mem[2]  - 8'h30;
						lon_min_dot_8_temp <= data_rx_mem[1]  - 8'h30;
						state_parse        <= GPGGA_LON_DIR;
					end
				end
				GPGGA_LON_DIR: begin
					if(data_rx_mem[0] == C_COMMA) begin
						if(data_rx_mem[1] == C_E) begin  //东经，0
							lon_dir_temp    <= 'b0;
						end
						else begin
							lon_dir_temp    <= 'b1;  //西经，1
						end
						state_parse  <=  GPGGA_ALT;
					end
				end
				GPGGA_ALT: begin
					if(data_rx_mem[0] == C_COMMA) begin
						//延迟4个,
						if(cnt_dly < 'd3) begin
							cnt_dly  <= cnt_dly + 'd1;
						end
						else begin
							cnt_dly  <= 'd0;
						end
						//取值
						if(cnt_dly == 'd3) begin
							//整数
							case(num_byte)
								'd1: begin
									alt_1_temp  <= 'd0;
									alt_2_temp  <= 'd0;
									alt_3_temp  <= 'd0;
									alt_4_temp  <= data_rx_mem[6] - 8'h30;
								end
								'd2: begin
									alt_1_temp  <= 'd0;
									alt_2_temp  <= 'd0;
									alt_3_temp  <= data_rx_mem[7] - 8'h30;
									alt_4_temp  <= data_rx_mem[6] - 8'h30;
								end
								'd3: begin
									alt_1_temp  <= 'd0;
									alt_2_temp  <= data_rx_mem[8] - 8'h30;
									alt_3_temp  <= data_rx_mem[7] - 8'h30;
									alt_4_temp  <= data_rx_mem[6] - 8'h30;
								end
								'd4: begin
									alt_1_temp  <= data_rx_mem[9] - 8'h30;
									alt_2_temp  <= data_rx_mem[8] - 8'h30;
									alt_3_temp  <= data_rx_mem[7] - 8'h30;
									alt_4_temp  <= data_rx_mem[6] - 8'h30;
								end
								default: begin
									alt_1_temp  <= 'd0;
									alt_2_temp  <= 'd0;
									alt_3_temp  <= 'd0;
									alt_4_temp  <= 'd0;
								end
							endcase
							//小数
							alt_dot_1_temp  <= data_rx_mem[4] - 8'h30;
							alt_dot_2_temp  <= data_rx_mem[3] - 8'h30;
							alt_dot_3_temp  <= data_rx_mem[2] - 8'h30;
							alt_dot_4_temp  <= data_rx_mem[1] - 8'h30;
							state_parse     <= GPGGA_END;
						end
						else begin
							alt_1_temp      <= 'd0;
							alt_2_temp      <= 'd0;
							alt_3_temp      <= 'd0;
							alt_4_temp      <= 'd0;
							alt_dot_1_temp  <= 'd0;
							alt_dot_2_temp  <= 'd0;
							alt_dot_3_temp  <= 'd0;
							alt_dot_4_temp  <= 'd0;
						end
					end

					//字节计数
					if(data_rx_mem[0] == C_COMMA) begin
						cnt_byte  <= 'd0;
					end
					else begin
						if(cnt_dly == 'd3) begin
							cnt_byte  <= cnt_byte + 'd1;
						end
						else begin
							cnt_byte  <= 'd0;
						end
					end

					//高度整数位数存储
					if(cnt_dly == 'd3 && data_rx_mem[0] == C_DOT) begin
						num_byte  <= cnt_byte;
					end
				end
				GPGGA_END: begin
					if(data_rx_mem[1] == C_CR && data_rx_mem[0] == C_LF) begin
						valid_gpgga  <= 'b1;
						state_parse  <= IDLE;
					end
				end
				GPZDA_DAY: begin
					if(data_rx_mem[0] == C_COMMA) begin
						//延迟1个,
						if(cnt_dly < 'd1) begin
							cnt_dly  <= cnt_dly + 'd1;
						end
						else begin
							cnt_dly  <= 'd0;
						end
						//取值
						if(cnt_dly == 'd1) begin
							day_1_temp     <= data_rx_mem[2] - 8'h30;
							day_2_temp     <= data_rx_mem[1] - 8'h30;
							state_parse    <= GPZDA_MONTH;
						end
					end
				end
				GPZDA_MONTH: begin
					if(data_rx_mem[0] == C_COMMA) begin
						mon_1_temp     <= data_rx_mem[2] - 8'h30;
						mon_2_temp     <= data_rx_mem[1] - 8'h30;
						state_parse    <= GPZDA_YEAR;
					end
				end
				GPZDA_YEAR: begin
					if(data_rx_mem[0] == C_COMMA) begin
						year_1_temp    <= data_rx_mem[4] - 8'h30;
						year_2_temp    <= data_rx_mem[3] - 8'h30;
						year_3_temp    <= data_rx_mem[2] - 8'h30;
						year_4_temp    <= data_rx_mem[1] - 8'h30;
						state_parse    <= GPZDA_END;
					end
				end
				GPZDA_END: begin
					if(data_rx_mem[1] == C_CR && data_rx_mem[0] == C_LF) begin
						valid_gpzda  <= 'b1;
						state_parse  <= IDLE;
					end
				end
				GNHPR_HEADING: begin
					if(data_rx_mem[0] == C_COMMA) begin
						//延迟1个,
						if(cnt_dly < 'd1) begin
							cnt_dly  <= cnt_dly + 'd1;
						end
						else begin
							cnt_dly  <= 'd0;
						end
						//取值
						if(cnt_dly == 'd1) begin
							heading_1_temp     <= data_rx_mem[6] - 8'h30;
							heading_2_temp     <= data_rx_mem[5] - 8'h30;
							heading_3_temp     <= data_rx_mem[4] - 8'h30;
							heading_dot_1_temp <= data_rx_mem[2] - 8'h30;
							heading_dot_2_temp <= data_rx_mem[1] - 8'h30;
							state_parse        <= GNHPR_END;
						end
					end
				end
				GNHPR_END: begin
					if(data_rx_mem[1] == C_CR && data_rx_mem[0] == C_LF) begin
						valid_gnhpr  <= 'b1;
						state_parse  <= IDLE;
					end
				end
			endcase
		end
	end
end

//--------------------------------------------------------------------
// 在有效脉冲下寄存数据
//--------------------------------------------------------------------
always @ (posedge i_clk_sys) begin
	if(valid_gpgga) begin
		utc_hour_1      <= utc_hour_1_temp     ;
		utc_hour_2      <= utc_hour_2_temp + DELTA_HOUR     ;
		utc_min_1       <= utc_min_1_temp      ;
		utc_min_2       <= utc_min_2_temp      ;
		utc_sec_1       <= utc_sec_1_temp      ;
		utc_sec_2       <= utc_sec_2_temp      ;
		utc_sec_dot_1   <= utc_sec_dot_1_temp  ;
		utc_sec_dot_2   <= utc_sec_dot_2_temp  ;
		lat_deg_1       <= lat_deg_1_temp      ;
		lat_deg_2       <= lat_deg_2_temp      ;
		lat_min_1       <= lat_min_1_temp      ;
		lat_min_2       <= lat_min_2_temp      ;
		lat_min_dot_1   <= lat_min_dot_1_temp  ;
		lat_min_dot_2   <= lat_min_dot_2_temp  ;
		lat_min_dot_3   <= lat_min_dot_3_temp  ;
		lat_min_dot_4   <= lat_min_dot_4_temp  ;
		lat_min_dot_5   <= lat_min_dot_5_temp  ;
		lat_min_dot_6   <= lat_min_dot_6_temp  ;
		lat_min_dot_7   <= lat_min_dot_7_temp  ;
		lat_min_dot_8   <= lat_min_dot_8_temp  ;
		lat_dir         <= lat_dir_temp        ;
		lon_deg_1       <= lon_deg_1_temp      ;
		lon_deg_2       <= lon_deg_2_temp      ;
		lon_deg_3       <= lon_deg_3_temp      ;
		lon_min_1       <= lon_min_1_temp      ;
		lon_min_2       <= lon_min_2_temp      ;
		lon_min_dot_1   <= lon_min_dot_1_temp  ;
		lon_min_dot_2   <= lon_min_dot_2_temp  ;
		lon_min_dot_3   <= lon_min_dot_3_temp  ;
		lon_min_dot_4   <= lon_min_dot_4_temp  ;
		lon_min_dot_5   <= lon_min_dot_5_temp  ;
		lon_min_dot_6   <= lon_min_dot_6_temp  ;
		lon_min_dot_7   <= lon_min_dot_7_temp  ;
		lon_min_dot_8   <= lon_min_dot_8_temp  ;
		lon_dir         <= lon_dir_temp        ;
		alt_1           <= alt_1_temp          ;
		alt_2           <= alt_2_temp          ;
		alt_3           <= alt_3_temp          ;
		alt_4           <= alt_4_temp          ;
		alt_dot_1       <= alt_dot_1_temp      ;
		alt_dot_2       <= alt_dot_2_temp      ;
		alt_dot_3       <= alt_dot_3_temp      ;
		alt_dot_4       <= alt_dot_4_temp      ;
	end
end

always @ (posedge i_clk_sys) begin
	if(valid_gpzda) begin
		day_1           <= day_1_temp          ;
		day_2           <= day_2_temp          ;
		mon_1           <= mon_1_temp          ;
		mon_2           <= mon_2_temp          ;
		year_1          <= year_1_temp         ;
		year_2          <= year_2_temp         ;
		year_3          <= year_3_temp         ;
		year_4          <= year_4_temp         ;
	end
end

always @ (posedge i_clk_sys) begin
	if(valid_gnhpr) begin
		heading_1           <= heading_1_temp      ;
		heading_2           <= heading_2_temp      ;
		heading_3           <= heading_3_temp      ;
		heading_dot_1       <= heading_dot_1_temp  ;
		heading_dot_2       <= heading_dot_2_temp  ;
	end
end

//有效脉冲延时
always @ (posedge i_clk_sys) begin
	valid_gpgga_r1  <= valid_gpgga;
end

//--------------------------------------------------------------------
// 对时间进行处理，转成地区时间
//--------------------------------------------------------------------
//每个月最后一天
always @ (posedge i_clk_sys) begin
	case({mon_1,mon_2})
		MON_1,MON_3,MON_5,MON_7,MON_8,MON_10,MON_12: begin
			flag_last_day  <= day_1 == 'd3 && day_2 == 'd1;
		end
		MON_2: begin
			if(year_4[1:0] == 'd0) begin  //闰年
				flag_last_day  <= day_1 == 'd2 && day_2 == 'd9;
			end
			else begin
				flag_last_day  <= day_1 == 'd2 && day_2 == 'd8;
			end
		end
		MON_4,MON_6,MON_9,MON_11: begin
			flag_last_day  <= day_1 == 'd3 && day_2 == 'd0;
		end
	endcase
end

//UTC转地区时间
always @ (posedge i_clk_sys) begin
	if(valid_gpgga_r1) begin
		if(year_4 == 'd9 && mon_1 == 'd1 && mon_2 == 'd2 && flag_last_day && utc_hour_1 == 'd1 && utc_hour_2 >= 'd14) begin
			year_3_r  <= year_3 + 'd1;
			year_4_r  <= 'd0;
			mon_1_r   <= 'd0;
			mon_2_r   <= 'd0;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd14;
		end
		else if(year_4 == 'd9 && mon_1 == 'd1 && mon_2 == 'd2 && flag_last_day && utc_hour_1 == 'd2 && utc_hour_2 >= 'd4) begin
			year_3_r  <= year_3 + 'd1;
			year_4_r  <= 'd0;
			mon_1_r   <= 'd0;
			mon_2_r   <= 'd0;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd4;
		end
		else if(mon_1 == 'd1 && mon_2 == 'd2 && flag_last_day && utc_hour_1 == 'd1 && utc_hour_2 >= 'd14) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4 + 'd1;
			mon_1_r   <= 'd0;
			mon_2_r   <= 'd0;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd14;
		end
		else if(mon_1 == 'd1 && mon_2 == 'd2 && flag_last_day && utc_hour_1 == 'd2 && utc_hour_2 >= 'd4) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4 + 'd1;
			mon_1_r   <= 'd0;
			mon_2_r   <= 'd0;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd4;
		end
		else if(mon_2 == 'd9 && flag_last_day && utc_hour_1 == 'd1 && utc_hour_2 >= 'd14) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1 + 'd1;
			mon_2_r   <= 'd0;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd14;
		end
		else if(mon_2 == 'd9 && flag_last_day && utc_hour_1 == 'd2 && utc_hour_2 >= 'd4) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1 + 'd1;
			mon_2_r   <= 'd0;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd4;
		end
		else if(flag_last_day && utc_hour_1 == 'd1 && utc_hour_2 >= 'd14) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2 + 'd1;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd14;
		end
		else if(flag_last_day && utc_hour_1 == 'd2 && utc_hour_2 >= 'd4) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2 + 'd1;
			day_1_r   <= 'd0;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd4;
		end
		else if(day_2 == 'd9 && utc_hour_1 == 'd1 && utc_hour_2 >= 'd14) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2;
			day_1_r   <= day_1 + 'd1;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd14;
		end
		else if(day_2 == 'd9 && utc_hour_1 == 'd2 && utc_hour_2 >= 'd4) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2;
			day_1_r   <= day_1 + 'd1;
			day_2_r   <= 'd0;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd4;
		end
		else if(utc_hour_1 == 'd1 && utc_hour_2 >= 'd14) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2;
			day_1_r   <= day_1;
			day_2_r   <= day_2 + 'd1;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd14;
		end
		else if(utc_hour_1 == 'd2 && utc_hour_2 >= 'd4) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2;
			day_1_r   <= day_1;
			day_2_r   <= day_2 + 'd1;
			hour_1_r  <= 'd0;
			hour_2_r  <= utc_hour_2 - 'd4;
		end
		else if(utc_hour_2 >= 'd10) begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2;
			day_1_r   <= day_1;
			day_2_r   <= day_2;
			hour_1_r  <= utc_hour_1 + 'd1;
			hour_2_r  <= utc_hour_2 - 'd10;
		end
		else begin
			year_3_r  <= year_3;
			year_4_r  <= year_4;
			mon_1_r   <= mon_1;
			mon_2_r   <= mon_2;
			day_1_r   <= day_1;
			day_2_r   <= day_2;
			hour_1_r  <= utc_hour_1;
			hour_2_r  <= utc_hour_2 + 'd1;
		end
		year_1_r  <= year_1;
		year_2_r  <= year_2;
	end
end

//分、秒
always @ (posedge i_clk_sys) begin
	if(valid_gpgga_r1) begin
		min_1_r       <= utc_min_1;
		min_2_r       <= utc_min_2;
		sec_1_r       <= utc_sec_1;
		sec_2_r       <= utc_sec_2;
		sec_dot_1_r   <= utc_sec_dot_1;
		sec_dot_2_r   <= utc_sec_dot_2;
	end
end

//--------------------------------------------------------------------
// 对地区时间进行处理，用PPS进行同步
//--------------------------------------------------------------------
//PPS到来时，当前秒内时间有效的标志，每一秒均进行复位
always @ (posedge i_clk_sys) begin
	if(valid_gpgga_r1) begin
		valid_time_in1s  <= 'b1;
	end
	else if(rs422_bd_pps_rxd_r2 && !rs422_bd_pps_rxd_r3) begin
		valid_time_in1s  <= 'b0;
	end
end

//每个月最后一天
always @ (posedge i_clk_sys) begin
	case({mon_1_r,mon_2_r})
		MON_1,MON_3,MON_5,MON_7,MON_8,MON_10,MON_12: begin
			flag_last_day_region  <= day_1_r == 'd3 && day_2_r == 'd1;
		end
		MON_2: begin
			if(year_4_r[1:0] == 'd0) begin  //闰年
				flag_last_day_region  <= day_1_r == 'd2 && day_2_r == 'd9;
			end
			else begin
				flag_last_day_region  <= day_1_r == 'd2 && day_2_r == 'd8;
			end
		end
		MON_4,MON_6,MON_9,MON_11: begin
			flag_last_day_region  <= day_1_r == 'd3 && day_2_r == 'd0;
		end
	endcase
end

//PPS上升沿进行同步
always @ (posedge i_clk_sys) begin
	if(valid_time_in1s && rs422_bd_pps_rxd_r2 && !rs422_bd_pps_rxd_r3) begin
		if(year_4_r == 'd9 && mon_1_r == 'd1 && mon_2_r == 'd2 && flag_last_day_region && hour_1_r == 'd2 && hour_2_r == 'd3 && min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r + 'd1;
			year_4_r2  <= 'd0;
			mon_1_r2   <= 'd0;
			mon_2_r2   <= 'd0;
			day_1_r2   <= 'd0;
			day_2_r2   <= 'd0;
			hour_1_r2  <= 'd0;
			hour_2_r2  <= 'd0;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(mon_1_r == 'd1 && mon_2_r == 'd2 && flag_last_day_region && hour_1_r == 'd2 && hour_2_r == 'd3 && min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r + 'd1;
			mon_1_r2   <= 'd0;
			mon_2_r2   <= 'd0;
			day_1_r2   <= 'd0;
			day_2_r2   <= 'd0;
			hour_1_r2  <= 'd0;
			hour_2_r2  <= 'd0;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(mon_2_r == 'd9 && flag_last_day_region && hour_1_r == 'd2 && hour_2_r == 'd3 && min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r + 'd1;
			mon_2_r2   <= 'd0;
			day_1_r2   <= 'd0;
			day_2_r2   <= 'd0;
			hour_1_r2  <= 'd0;
			hour_2_r2  <= 'd0;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(flag_last_day_region && hour_1_r == 'd2 && hour_2_r == 'd3 && min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r + 'd1;
			day_1_r2   <= 'd0;
			day_2_r2   <= 'd0;
			hour_1_r2  <= 'd0;
			hour_2_r2  <= 'd0;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(day_2_r == 'd9 && hour_1_r == 'd2 && hour_2_r == 'd3 && min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r + 'd1;
			day_2_r2   <= 'd0;
			hour_1_r2  <= 'd0;
			hour_2_r2  <= 'd0;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(hour_1_r == 'd2 && hour_2_r == 'd3 && min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r;
			day_2_r2   <= day_2_r + 'd1;
			hour_1_r2  <= 'd0;
			hour_2_r2  <= 'd0;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(hour_2_r == 'd9 && min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r;
			day_2_r2   <= day_2_r;
			hour_1_r2  <= hour_1_r + 'd1;
			hour_2_r2  <= 'd0;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(min_1_r == 'd5 && min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r;
			day_2_r2   <= day_2_r;
			hour_1_r2  <= hour_1_r;
			hour_2_r2  <= hour_2_r + 'd1;
			min_1_r2   <= 'd0;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(min_2_r == 'd9 && sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r;
			day_2_r2   <= day_2_r;
			hour_1_r2  <= hour_1_r;
			hour_2_r2  <= hour_2_r;
			min_1_r2   <= min_1_r + 'd1;
			min_2_r2   <= 'd0;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(sec_1_r == 'd5 && sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r;
			day_2_r2   <= day_2_r;
			hour_1_r2  <= hour_1_r;
			hour_2_r2  <= hour_2_r;
			min_1_r2   <= min_1_r;
			min_2_r2   <= min_2_r + 'd1;
			sec_1_r2   <= 'd0;
			sec_2_r2   <= 'd0;
		end
		else if(sec_2_r == 'd9) begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r;
			day_2_r2   <= day_2_r;
			hour_1_r2  <= hour_1_r;
			hour_2_r2  <= hour_2_r;
			min_1_r2   <= min_1_r;
			min_2_r2   <= min_2_r;
			sec_1_r2   <= sec_1_r + 'd1;
			sec_2_r2   <= 'd0;
		end
		else begin
			year_3_r2  <= year_3_r;
			year_4_r2  <= year_4_r;
			mon_1_r2   <= mon_1_r;
			mon_2_r2   <= mon_2_r;
			day_1_r2   <= day_1_r;
			day_2_r2   <= day_2_r;
			hour_1_r2  <= hour_1_r;
			hour_2_r2  <= hour_2_r;
			min_1_r2   <= min_1_r;
			min_2_r2   <= min_2_r;
			sec_1_r2   <= sec_1_r;
			sec_2_r2   <= sec_2_r + 'd1;
		end
	end
end

//经过PPS同步后的时间有效标志
//1.有时间，没有PPS，该标志不会置位；
//2.没时间，有PPS，该标志不会置位；
//3.有时间，有PPS该标志会置位
//在授时不稳定时，采用内部自动守时
always @ (posedge i_clk_sys) begin
	if(valid_time_in1s && rs422_bd_pps_rxd_r2 && !rs422_bd_pps_rxd_r3) begin
		valid_time  <= 'b1;
	end
	else begin
		valid_time  <= 'b0;
	end
end

//--------------------------------------------------------------------
// degbug
//--------------------------------------------------------------------
//reg  [31:0]  cnt_ms  = 'd0;
//reg          flag_ms = 'b0;
//always @ (posedge i_clk_sys) begin
//	if(cnt_ms < 'd99999) begin
//		cnt_ms  <= cnt_ms + 'd1;
//		flag_ms <= 'b0;
//	end
//	else begin
//		cnt_ms  <= 'd0;
//		flag_ms <= 'b1;
//	end
//end
//
//ila_0 u_ila_0 (
//	.clk     (i_clk_sys           ), // input wire clk
//	.probe0  (year_1_r            ), // input wire [7:0]  probe0
//	.probe1  (year_2_r            ), // input wire [7:0]  probe1
//	.probe2  (year_3_r            ), // input wire [7:0]  probe2
//	.probe3  (year_4_r            ), // input wire [7:0]  probe3
//	.probe4  (mon_1_r             ), // input wire [7:0]  probe4
//	.probe5  (mon_2_r             ), // input wire [7:0]  probe5
//	.probe6  (day_1_r             ), // input wire [7:0]  probe6
//	.probe7  (day_2_r             ), // input wire [7:0]  probe7
//	.probe8  (hour_1_r            ), // input wire [7:0]  probe8
//	.probe9  (hour_2_r            ), // input wire [7:0]  probe9
//	.probe10 (min_1_r             ), // input wire [7:0]  probe10
//	.probe11 (min_2_r             ), // input wire [7:0]  probe11
//	.probe12 (sec_1_r             ), // input wire [7:0]  probe12
//	.probe13 (sec_2_r             ), // input wire [7:0]  probe13
//	.probe14 (sec_dot_1_r         ), // input wire [7:0]  probe14
//	.probe15 (sec_dot_2_r         ), // input wire [7:0]  probe15
//	.probe16 (rs422_bd_pps_rxd_r2 ), // input wire [0:0]  probe16
//	.probe17 (flag_ms             ), // input wire [0:0]  probe17
//	.probe18 (lat_deg_1           ), // input wire [7:0]  probe18
//	.probe19 (lat_deg_2           ), // input wire [7:0]  probe19
//	.probe20 (lat_min_1           ), // input wire [7:0]  probe20
//	.probe21 (lat_min_2           ), // input wire [7:0]  probe21
//	.probe22 (lat_min_dot_1       ), // input wire [7:0]  probe22
//	.probe23 (lat_min_dot_2       ), // input wire [7:0]  probe23
//	.probe24 (lat_min_dot_3       ), // input wire [7:0]  probe24
//	.probe25 (lat_min_dot_4       ), // input wire [7:0]  probe25
//	.probe26 (lat_min_dot_5       ), // input wire [7:0]  probe26
//	.probe27 (lat_min_dot_6       ), // input wire [7:0]  probe27
//	.probe28 (lat_min_dot_7       ), // input wire [7:0]  probe28
//	.probe29 (lat_min_dot_8       ), // input wire [7:0]  probe29
//	.probe30 (lat_dir             ), // input wire [7:0]  probe30
//	.probe31 (lon_deg_1           ), // input wire [7:0]  probe31
//	.probe32 (lon_deg_2           ), // input wire [7:0]  probe32
//	.probe33 (lon_deg_3           ), // input wire [7:0]  probe33
//	.probe34 (lon_min_1           ), // input wire [7:0]  probe34
//	.probe35 (lon_min_2           ), // input wire [7:0]  probe35
//	.probe36 (lon_min_dot_1       ), // input wire [7:0]  probe36
//	.probe37 (lon_min_dot_2       ), // input wire [7:0]  probe37
//	.probe38 (lon_min_dot_3       ), // input wire [7:0]  probe38
//	.probe39 (lon_min_dot_4       ), // input wire [7:0]  probe39
//	.probe40 (lon_min_dot_5       ), // input wire [7:0]  probe40
//	.probe41 (lon_min_dot_6       ), // input wire [7:0]  probe41
//	.probe42 (lon_min_dot_7       ), // input wire [7:0]  probe42
//	.probe43 (lon_min_dot_8       ), // input wire [7:0]  probe43
//	.probe44 (lon_dir             ), // input wire [7:0]  probe44
//	.probe45 (heading_1           ), // input wire [7:0]  probe45
//	.probe46 (heading_2           ), // input wire [7:0]  probe46
//	.probe47 (heading_3           ), // input wire [7:0]  probe47
//	.probe48 (heading_dot_1       ), // input wire [7:0]  probe48
//	.probe49 (heading_dot_2       ), // input wire [7:0]  probe49
//	.probe50 (alt_1               ), // input wire [7:0]  probe50
//	.probe51 (alt_2               ), // input wire [7:0]  probe51
//	.probe52 (alt_3               ), // input wire [7:0]  probe52
//	.probe53 (alt_4               ), // input wire [7:0]  probe53
//	.probe54 (alt_dot_1           ), // input wire [7:0]  probe54
//	.probe55 (alt_dot_2           ), // input wire [7:0]  probe55
//	.probe56 (alt_dot_3           ), // input wire [7:0]  probe56
//	.probe57 (alt_dot_4           )  // input wire [7:0]  probe57
//);

endmodule


//general control

module uart #(
    parameter BAUD_RATE    = 115200, //波特率，每秒发送的二进制位数（bits）
    parameter CLK_FRE      = 50000000, //时钟频率
    parameter PAYLOAD_BITS = 8 //数据位数
) (
    input                          clk,
    input                          rst_n,
    input [2*PAYLOAD_BITS - 1 : 0] cmd_in,
    input                          cmd_vld,
    output                         read_vld,
    output [PAYLOAD_BITS - 1 : 0]  read_data,
    output                         cmd_rdy
);

wire connect1,connect2;
wire read_vld_s;
wire data_rdy;
wire [2*PAYLOAD_BITS - 1 : 0] data_in_mem;
wire [PAYLOAD_BITS - 1   : 0] data_out_mem;

master_tx u_master_tx(
    .clk(clk),
    .rst_n(rst_n),
    .cmd_in(cmd_in),
    .cmd_vld(cmd_vld),
    .tx_m(connect1),
    .cmd_rdy_tx(cmd_rdy)
    );

master_rx u_master_rx(
    .clk(clk),
    .rst_n(rst_n),
    .rx_m(connect2),
    .cmd_vld(cmd_vld),
    .read_vld(read_vld),
    .read_data(read_data)
    );

slave_rx u_slave_rx(
    .clk(clk),
    .rst_n(rst_n),
    .rx_s(connect1),
    .data_in(data_in_mem),
    .data_rdy(data_rdy)
    );

slave_tx u_slave_tx(
    .clk(clk),
    .rst_n(rst_n),
    .data_out(data_out_mem),
    .tx_s(connect2),
    .read_vld_s(read_vld_s)
    );
    
slave_mem u_slave_mem(
    .data_in(data_in_mem),
    .clk(clk),
    .rst_n(rst_n),
    .data_out(data_out_mem),
    .read_vld_s(read_vld_s),
    .data_rdy(data_rdy)
    );
    
endmodule

//master_tx
module master_tx #(
    parameter BAUD_RATE    = 115200,
    parameter CLK_FRE      = 50000000,
    parameter PAYLOAD_BITS = 8
) (
    input                          clk,
    input                          rst_n,
    input [2*PAYLOAD_BITS - 1 : 0] cmd_in,
    input                          cmd_vld,
    output                         tx_m,
    output                         cmd_rdy_tx
);

    reg r_tx_m;
    reg r_cmd_rdy_tx;
    assign cmd_rdy_tx = r_cmd_rdy_tx;
    assign tx_m       = r_tx_m;

    localparam CLKS_PER_BITS = CLK_FRE / BAUD_RATE; //每发1bit数据需要多少拍
    localparam CLK_CNT_BW    = $clog2(CLKS_PER_BITS) + 1; //拍数计数器的位宽
    localparam BITS_CNT_BW   = $clog2(PAYLOAD_BITS) + 1;  //数据计数器的位宽

    reg start_bit_ini;
    reg data_bit_ini;
    reg parity_bit_ini;
    reg stop_bit_ini;
    reg stop_bit_end;
    reg delay_bit_end;

    reg [2*PAYLOAD_BITS - 1 : 0] r_cmd_in;

    reg [CLK_CNT_BW   : 0]  clk_cnt;
    reg [BITS_CNT_BW  : 0] bits_cnt;
    reg [1 : 0]               state_cnt; //因为写的话要先写地址再写数据，所以引入变量来控制第二次的写数据

    enum logic [2 : 0] {IDLE,START,DATA,PARITY,STOP_WR,STOP_RD,DELAY} current_state,next_state;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            current_state <= IDLE;
        end
        else begin
            current_state <= next_state;
        end
    end

    always @(*) begin
        next_state = IDLE;
        case(current_state)
            IDLE: begin
                if(start_bit_ini == 1'b1) begin
                    next_state = START;
                end
                else begin
                    next_state = IDLE;
                end
            end 
            START: begin
                if(data_bit_ini == 1'b1) begin
                    next_state = DATA;
                end
                else begin
                    next_state = START;
                end
            end
            DATA: begin
                if(parity_bit_ini == 1'b1) begin
                    next_state = PARITY;
                end
                else begin
                    next_state = DATA;
                end
            end
            PARITY: begin
                if(stop_bit_ini == 1'b1) begin
                    if(r_cmd_in[2*PAYLOAD_BITS - 1] == 1'b1) begin
                        next_state = STOP_WR;
                    end
                    else begin
                        next_state = STOP_RD;
                    end
                end
                else begin
                    next_state = PARITY;
                end
            end
            STOP_WR: begin
                if(stop_bit_end == 1'b1) begin
                    next_state = DELAY;
                end
                else begin
                    next_state = STOP_WR;
                end
            end
            STOP_RD: begin
                if(stop_bit_end == 1'b1) begin
                    next_state = IDLE;
                end
                else begin
                    next_state = STOP_RD;
                end
            end
            DELAY: begin
                if(delay_bit_end == 1'b1) begin
                    next_state = IDLE;
                end
                else begin
                    next_state = DELAY;
                end
            end
        endcase
    end

   always @( *) begin
        start_bit_ini  = 0;
        data_bit_ini   = 0;
        parity_bit_ini = 0;
        stop_bit_ini   = 0;
        stop_bit_end   = 0;
        delay_bit_end  = 0;
        r_cmd_rdy_tx   = 0; 
        case(current_state)
            IDLE: begin
                r_cmd_rdy_tx  = state_cnt == 2'b00;
                start_bit_ini = (cmd_vld == 1'b1 && r_cmd_rdy_tx == 1'b1) || state_cnt == 2'b01;
            end
            START: begin
                data_bit_ini = clk_cnt == CLKS_PER_BITS;
            end
            DATA: begin
                if(bits_cnt < PAYLOAD_BITS) begin
                    data_bit_ini = clk_cnt == CLKS_PER_BITS;
                end
                else begin
                    parity_bit_ini = clk_cnt == CLKS_PER_BITS;
                end
            end
            PARITY: begin
                stop_bit_ini = clk_cnt == CLKS_PER_BITS;
            end
            STOP_WR: begin
                stop_bit_end = clk_cnt == CLKS_PER_BITS;
            end
            STOP_RD: begin
                stop_bit_end = clk_cnt == CLKS_PER_BITS;
            end
            DELAY: begin
                delay_bit_end = clk_cnt == 2*PAYLOAD_BITS;
            end
        endcase
   end 

   always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            clk_cnt   <= {CLK_CNT_BW{1'b0}};
            bits_cnt  <= {BITS_CNT_BW{1'b0}};
            state_cnt <= 2'b00;
        end
        else begin
            case(current_state)
                IDLE: begin
                    clk_cnt  <= {CLK_CNT_BW{1'b0}};
                    bits_cnt <= {BITS_CNT_BW{1'b0}};
                    state_cnt <= state_cnt == 2'b10 ? 0 : state_cnt;
                end
                START: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end

                    if(data_bit_ini == 1'b1) begin
                        bits_cnt <= bits_cnt + 1'b1;
                    end
                end
                DATA: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end

                    if(data_bit_ini == 1'b1) begin
                        if(bits_cnt < PAYLOAD_BITS) begin
                            bits_cnt <= bits_cnt + 1'b1;
                        end
                        else begin
                            bits_cnt <= {BITS_CNT_BW{1'b0}};
                        end
                    end

                    if(parity_bit_ini == 1'b1) begin
                        state_cnt <= state_cnt + 1'b1;
                    end
                end
                PARITY: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                STOP_WR: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                STOP_RD: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                DELAY: begin
                    if(clk_cnt < 2*CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
            endcase
        end
   end

   always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            r_tx_m <= 1'b1;
        end
        else begin
            if(start_bit_ini == 1'b1) begin
                r_tx_m <= 1'b0;
            end
            else if(state_cnt == 2'b00 && data_bit_ini == 1'b1) begin
                r_tx_m <= r_cmd_in[2*PAYLOAD_BITS-1-bits_cnt];
            end
            else if(state_cnt == 2'b01 && data_bit_ini == 1) begin
                r_tx_m <= r_cmd_in[PAYLOAD_BITS-1-bits_cnt];
            end
            else if(state_cnt == 2'b00 && parity_bit_ini) begin
                r_tx_m <= ^r_cmd_in[2*PAYLOAD_BITS-1 : PAYLOAD_BITS];
            end
            else if(state_cnt == 2'b01 && parity_bit_ini) begin
                r_tx_m <= ^r_cmd_in[PAYLOAD_BITS-1 : 0];
            end
            else if(stop_bit_ini) begin
                r_tx_m <= 1'b1;
            end
        end
   end

endmodule

//master_rx
module master_rx #(
    parameter BAUD_RATE    = 115200,
    parameter CLK_FRE      = 50000000,
    parameter PAYLOAD_BITS = 8
) (
    input                          clk,
    input                          rst_n,
    input                          rx_m,
    input                          cmd_vld,
    output                         read_vld,
    output [PAYLOAD_BITS - 1 : 0]  read_data
);

    reg [PAYLOAD_BITS - 1 : 0] r_read_data;
    reg                        r_read_vld;
    assign read_vld  = r_read_vld;
    assign read_data = r_read_vld == 1'b1 ? r_read_data : 0;

    localparam CLKS_PER_BITS = CLK_FRE / BAUD_RATE; //每发1bit数据需要多少拍
    localparam CLK_CNT_BW    = $clog2(CLKS_PER_BITS) + 1; //拍数计数器的位宽
    localparam BITS_CNT_BW   = $clog2(PAYLOAD_BITS) + 1;  //数据计数器的位宽

    reg start_bit_ini;
    reg start_bit_mid;
    reg data_bit_mid;
    reg parity_bit_mid;
    reg stop_bit_mid;
    // reg r_rx_m;

    reg [CLK_CNT_BW  : 0]  clk_cnt;
    reg [BITS_CNT_BW : 0]  bits_cnt;

    enum logic [2 : 0] {IDLE,START,DATA,PARITY,STOP} current_state,next_state;

    // always @(posedge clk or negedge rst_n) begin
    //     if(!rst_n) begin
    //         r_rx_m <= 1'b0;
    //     end
    //     else begin
    //         r_rx_m <= rx_m;
    //     end
    // end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            current_state <= IDLE;
        end
        else begin
            current_state <= next_state;
        end
    end

    always @(*) begin
        next_state = IDLE;
        case(current_state)
            IDLE: begin
                if(start_bit_ini == 1'b1) begin
                    next_state = START;
                end
                else begin
                    next_state = IDLE;
                end
            end 
            START: begin
                if(start_bit_mid == 1'b1) begin
                    next_state = DATA;
                end
                else begin
                    next_state = START;
                end
            end
            DATA: begin
                if(bits_cnt < PAYLOAD_BITS) begin
                    next_state = DATA;
                end
                else begin
                    next_state = PARITY;
                end
            end
            PARITY: begin
                if(parity_bit_mid == 1'b1) begin
                    next_state = STOP;
                end
                else begin
                    next_state = PARITY;
                end
            end
            STOP: begin
                if(stop_bit_mid) begin
                    next_state = IDLE;
                end
                else begin
                    next_state = STOP;
                end
            end
        endcase
    end

   always @( *) begin
        start_bit_ini  = 0;
        start_bit_mid  = 0;
        data_bit_mid   = 0;
        parity_bit_mid = 0;
        stop_bit_mid   = 0;
        case(current_state)
            IDLE: begin
                start_bit_ini = rx_m == 1'b0;
            end
            START: begin
                start_bit_mid = clk_cnt == CLKS_PER_BITS/2;
            end
            DATA: begin
                data_bit_mid = clk_cnt == CLKS_PER_BITS;
            end
            PARITY: begin
                parity_bit_mid = clk_cnt == CLKS_PER_BITS;
            end
            STOP: begin
                stop_bit_mid = clk_cnt == CLKS_PER_BITS;
            end
        endcase
   end 

   always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            clk_cnt   <= {CLK_CNT_BW{1'b0}};
            bits_cnt  <= {BITS_CNT_BW{1'b0}};
        end
        else begin
            case(current_state)
                IDLE: begin
                    clk_cnt  <= {CLK_CNT_BW{1'b0}};
                    bits_cnt <= {BITS_CNT_BW{1'b0}};
                end
                START: begin
                    if(clk_cnt < CLKS_PER_BITS/2) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                DATA: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end

                    if(data_bit_mid == 1'b1) begin
                        if(bits_cnt < PAYLOAD_BITS) begin
                            bits_cnt <= bits_cnt + 1'b1;
                        end
                        else if(parity_bit_mid == 1'b1) begin
                            bits_cnt <= {BITS_CNT_BW{1'b0}};
                        end
                    end
                end
                PARITY: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                STOP: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
            endcase
        end
   end

   always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin  
        end
        else if(data_bit_mid) begin
            r_read_data[7-bits_cnt] <= rx_m;
        end
   end

   always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r_read_vld <= 1'b0;
        end
        else begin
            r_read_vld <= current_state == IDLE;
        end
   end  

endmodule

//slave_rx
module slave_rx #(
    parameter BAUD_RATE    = 115200,
    parameter CLK_FRE      = 50000000,
    parameter PAYLOAD_BITS = 8
) (
    input                           clk,
    input                           rst_n,
    input                           rx_s,
    output [2*PAYLOAD_BITS - 1 : 0] data_in,
    output                          data_rdy
);

    localparam CLKS_PER_BITS = CLK_FRE / BAUD_RATE; //每发1bit数据需要多少拍
    localparam CLK_CNT_BW    = $clog2(CLKS_PER_BITS) + 1; //拍数计数器的位宽
    localparam BITS_CNT_BW   = $clog2(PAYLOAD_BITS) + 1;  //数据计数器的位宽

    reg                          start_bit_ini;
    reg                          start_bit_mid;
    reg                          data_bit_mid;
    reg                          parity_bit_mid;
    reg                          stop_bit_mid;
    reg [2*PAYLOAD_BITS - 1 : 0] r_data_in;           
    // reg r_rx_s;

    reg [CLK_CNT_BW  : 0]  clk_cnt;
    reg [BITS_CNT_BW : 0]  bits_cnt;
    reg [1 : 0]            state_cnt;
    reg                    r_data_rdy;

    assign data_in  = r_data_in;
    assign data_rdy = r_data_rdy;

    enum logic [2 : 0] {IDLE,START,DATA,PARITY,STOP_WR,STOP_RD} current_state,next_state;


    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            current_state <= IDLE;
        end
        else begin
            current_state <= next_state;
        end
    end

    always @(*) begin
        next_state = IDLE;
        case(current_state)
            IDLE: begin
                if(start_bit_ini == 1'b1) begin
                    next_state = START;
                end
                else begin
                    next_state = IDLE;
                end
            end 
            START: begin
                if(start_bit_mid == 1'b1) begin
                    next_state = DATA;
                end
                else begin
                    next_state = START;
                end
            end
            DATA: begin
                if(bits_cnt < PAYLOAD_BITS) begin
                    next_state = DATA;
                end
                else begin
                    next_state = PARITY;
                end
            end
            PARITY: begin
                if(parity_bit_mid == 1'b1) begin
                    if(r_data_in[15] == 1) begin
                        next_state = STOP_WR;
                    end
                    else begin
                        next_state = STOP_RD;
                    end
                end
                else begin
                    next_state = PARITY;
                end
            end
            STOP_WR: begin
                if(stop_bit_mid) begin
                    next_state = IDLE;
                end
                else begin
                    next_state = STOP_WR;
                end
            end
            STOP_RD: begin
                if(stop_bit_mid) begin
                    next_state = IDLE;
                end
                else begin
                    next_state = STOP_RD;
                end
            end
        endcase
    end

   always @( *) begin
        start_bit_ini  = 0;
        start_bit_mid  = 0;
        data_bit_mid   = 0;
        parity_bit_mid = 0;
        stop_bit_mid   = 0;
        case(current_state)
            IDLE: begin
                start_bit_ini = rx_m == 1'b0;
            end
            START: begin
                start_bit_mid = clk_cnt == CLKS_PER_BITS/2;
            end
            DATA: begin
                data_bit_mid = clk_cnt == CLKS_PER_BITS;
            end
            PARITY: begin
                parity_bit_mid = clk_cnt == CLKS_PER_BITS;
            end
            STOP_WR: begin
                stop_bit_mid = clk_cnt == CLKS_PER_BITS;
            end
            STOP_RD: begin
                stop_bit_mid = clk_cnt == CLKS_PER_BITS;
            end
        endcase
   end 

   always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            clk_cnt   <= {CLK_CNT_BW{1'b0}};
            bits_cnt  <= {BITS_CNT_BW{1'b0}};
            state_cnt <= 2'b00;
        end
        else begin
            case(current_state)
                IDLE: begin
                    clk_cnt  <= {CLK_CNT_BW{1'b0}};
                    bits_cnt <= {BITS_CNT_BW{1'b0}};
                    state_cnt <= state_cnt == 2'b10 ? 2'b00 : state_cnt;
                end
                START: begin
                    if(clk_cnt < CLKS_PER_BITS/2) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                DATA: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end

                    if(data_bit_mid == 1'b1) begin
                        if(bits_cnt < PAYLOAD_BITS) begin
                            bits_cnt <= bits_cnt + 1'b1;
                        end
                        else if(parity_bit_mid == 1'b1) begin
                            bits_cnt <= {BITS_CNT_BW{1'b0}};
                        end
                    end

                    if(bits_cnt == 8) begin
                        state_cnt <= state_cnt + 1'b1;
                    end
                end
                PARITY: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                STOP_WR: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                STOP_RD: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
            endcase
        end
   end

   always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            r_data_rdy <= 1'b0;  
        end
        else if(data_bit_mid) begin
            if(state_cnt == 2'b00) begin
                r_data_in[15-bits_cnt] <= rx_s;
            end
            else if(state_cnt == 2'b01) begin
                r_data_in[7-bits_cnt] <= rx_s;
            end
        end
        else if(current_state == IDLE) begin
            r_data_rdy <= ((state_cnt == 2'b10 && r_data_in[15] == 1'b1) || (state_cnt == 2'b01 && r_data_in[15] == 1'b0));
        end
        else begin
            r_data_rdy <= 1'b0;
        end
   end


endmodule

//slave_tx
module slave_tx #(
    parameter BAUD_RATE    = 115200,
    parameter CLK_FRE      = 50000000,
    parameter PAYLOAD_BITS = 8
) (
    input                          clk,
    input                          rst_n,
    input [PAYLOAD_BITS - 1 : 0]   data_out,
    input                          read_vld_s,
    output                         tx_s
);


    localparam CLKS_PER_BITS = CLK_FRE / BAUD_RATE; //每发1bit数据需要多少拍
    localparam CLK_CNT_BW    = $clog2(CLKS_PER_BITS) + 1; //拍数计数器的位宽
    localparam BITS_CNT_BW   = $clog2(PAYLOAD_BITS) + 1;  //数据计数器的位宽

    reg                        start_bit_ini;
    reg                        data_bit_ini;
    reg                        parity_bit_ini;
    reg                        stop_bit_ini;
    reg                        stop_bit_end;
    reg                        r_tx_s;
    reg [PAYLOAD_BITS - 1 : 0] r_data_out;

    reg [CLK_CNT_BW   : 0] clk_cnt;
    reg [BITS_CNT_BW  : 0] bits_cnt;

    enum logic [2 : 0] {IDLE,START,DATA,PARITY,STOP} current_state,next_state;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            current_state <= IDLE;
        end
        else begin
            current_state <= next_state;
        end
    end

    always @(*) begin
        next_state = IDLE;
        case(current_state)
            IDLE: begin
                if(start_bit_ini == 1'b1) begin
                    next_state = START;
                end
                else begin
                    next_state = IDLE;
                end
            end 
            START: begin
                if(data_bit_ini == 1'b1) begin
                    next_state = DATA;
                end
                else begin
                    next_state = START;
                end
            end
            DATA: begin
                if(parity_bit_ini == 1'b1) begin
                    next_state = PARITY;
                end
                else begin
                    next_state = DATA;
                end
            end
            PARITY: begin
                if(stop_bit_ini == 1'b1) begin
                    next_state = STOP;
                end
                else begin
                    next_state = PARITY;
                end
            end
            STOP: begin
                if(stop_bit_end == 1'b1) begin
                    next_state = IDLE;
                end
                else begin
                    next_state = STOP;
                end
            end
        endcase
    end

   always @( *) begin
        start_bit_ini  = 0;
        data_bit_ini   = 0;
        parity_bit_ini = 0;
        stop_bit_ini   = 0;
        stop_bit_end   = 0;
        case(current_state)
            IDLE: begin
                start_bit_ini = read_vld_s == 1'b1;
            end
            START: begin
                data_bit_ini = clk_cnt == CLKS_PER_BITS;
            end
            DATA: begin
                if(bits_cnt < PAYLOAD_BITS) begin
                    data_bit_ini = clk_cnt == CLKS_PER_BITS;
                end
                else begin
                    parity_bit_ini = clk_cnt == CLKS_PER_BITS;
                end
            end
            PARITY: begin
                stop_bit_ini = clk_cnt == CLKS_PER_BITS;
            end
            STOP: begin
                stop_bit_end = clk_cnt == CLKS_PER_BITS;
            end
        endcase
   end 

   always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            clk_cnt   <= {CLK_CNT_BW{1'b0}};
            bits_cnt  <= {BITS_CNT_BW{1'b0}};
        end
        else begin
            case(current_state)
                IDLE: begin
                    clk_cnt  <= {CLK_CNT_BW{1'b0}};
                    bits_cnt <= {BITS_CNT_BW{1'b0}};
                end
                START: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end

                    if(data_bit_ini == 1'b1) begin
                        bits_cnt <= bits_cnt + 1'b1;
                    end
                end
                DATA: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end

                    if(data_bit_ini == 1'b1) begin
                        bits_cnt <= bits_cnt + 1'b1;
                    end
                    else if(parity_bit_ini) begin
                        bits_cnt <= {BITS_CNT_BW{1'b0}};
                    end
                end
                PARITY: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
                STOP: begin
                    if(clk_cnt < CLKS_PER_BITS) begin
                        clk_cnt <= clk_cnt + 1'b1;
                    end
                    else begin
                        clk_cnt <= {CLK_CNT_BW{1'b0}};
                    end
                end
            endcase
        end
   end

   always @(posedge clk or negedge rst_n) begin
        if(~rst_n) begin
            r_tx_s <= 1'b1;
        end
        else begin
            if(start_bit_ini == 1'b1) begin
                r_tx_s <= 1'b0;
            end
            else if(data_bit_ini == 1'b1) begin
                r_tx_s <= r_data_out[7-bits_cnt];
            end
            else if(parity_bit_ini) begin
                r_tx_s <= ^r_data_out;
            end
            else if(stop_bit_ini) begin
                r_tx_s <= 1'b1;
            end
        end
   end

   always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
        end
        else if (current_state == IDLE) begin
            r_data_out <= data_out;
        end
   end

    assign tx_s = r_tx_s;

endmodule

//slave_mem
module slave_mem #(
    parameter BAUD_RATE    = 115200,
    parameter CLK_FRE      = 50000000,
    parameter PAYLOAD_BITS = 8 
) (
    input                          data_rdy,
    input [2*PAYLOAD_BITS - 1 : 0] data_in,
    input                          clk,
    input                          rst_n,
    output [PAYLOAD_BITS - 1 : 0]  data_out,
    output                         read_vld_s
);

    reg [PAYLOAD_BITS - 1 : 0] r_data_out;
    assign data_out = r_data_out;

    reg [PAYLOAD_BITS - 1 : 0]   mem [127 : 0];
    reg [2*PAYLOAD_BITS - 1 : 0] r_data_in;
    reg                          r_read_vld_s;
    wire [PAYLOAD_BITS - 2 : 0]  addr;
    wire [PAYLOAD_BITS - 1 : 0]  data;

    assign addr = r_data_in[2*PAYLOAD_BITS - 2 : PAYLOAD_BITS];
    assign data = r_data_in[PAYLOAD_BITS - 1 : 0];
    assign read_vld_s = r_read_vld_s;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
        end
        else if(data_rdy) begin
            r_data_in <= data_in;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
        end
        else if(r_data_in[2*PAYLOAD_BITS - 1] == 1'b0) begin
            r_data_out <= mem[addr];
            r_read_vld_s <= 1'b1;
        end
        else if(r_data_in[2*PAYLOAD_BITS - 1]) begin
            r_read_vld_s <= 1'b0;
        end
    end
    
    always @(posedge clk or negedge rst_n) begin
        if(r_data_in[2*PAYLOAD_BITS - 1]) begin
            mem[addr] <= data;
        end
    end
endmodule
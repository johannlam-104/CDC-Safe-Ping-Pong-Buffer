module ping_pong_formal_wrap;
    // master clock
    reg fclk = 0;
    always #1 fclk = ~fclk;

    // derived clocks
    reg i_wclk = 0;
    reg i_rclk = 0;

    // nondeterministic clock enables
    (* anyseq *) reg w_en;
    (* anyseq *) reg r_en;

    reg f_past_valid;
    reg w_en_d, r_en_d;
    
    initial begin
      f_past_valid = 1'b0;
      w_en_d = 1'b0;
      r_en_d = 1'b0;
    end
    
    always @(posedge fclk) begin
      f_past_valid <= 1'b1;
      w_en_d <= w_en;
      r_en_d <= r_en;
    end


    always @(posedge fclk) begin
        if (w_en) i_wclk <= ~i_wclk;
        if (r_en) i_rclk <= ~i_rclk;
    end

    // Fairness assumptions: clocks must eventually toggle
    always @(posedge fclk) begin
        assume(w_en || w_en_d);
        assume(r_en || r_en_d);
    end
    
    
    // stimulus
    (* anyseq *) reg i_s_valid;
    (* anyseq *) reg i_s_last;
    (* anyseq *) reg [11:0] i_s_data;
    (* anyseq *) reg i_m_ready;
    (* anyseq *) reg i_swap_ok;

    // reset
    reg i_wrstn = 0;
    reg i_rrstn = 0;

    initial begin
        #5 i_wrstn = 1;
        #5 i_rrstn = 1;
    end

    // DUT
    ping_pong_buf dut (
        .i_wclk(i_wclk),
        .i_wrstn(i_wrstn),
        .i_s_valid(i_s_valid),
        .i_s_data(i_s_data),
        .i_s_last(i_s_last),
        .o_s_ready(),

        .i_rclk(i_rclk),
        .i_rrstn(i_rrstn),
        .o_m_valid(),
        .i_m_ready(i_m_ready),
        .o_m_data(),
        .o_m_last(),
        .i_swap_ok(i_swap_ok)
    );

endmodule

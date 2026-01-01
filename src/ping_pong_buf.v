// ==============================================
// A CDC safe ping pong buffering system 
// with formal verification proof that spans
// both bounded safety proofs and liveness
// ---------------------------------------------
// Features
// * AXI-Stream I/O for variable length streaming frames
// * Optional framebuffer addressing for video applications
// ==============================================

module ping_pong_buf #(
    parameter FRAME_WIDTH  = 320,
    parameter FRAME_HEIGHT = 240,
    parameter DATA_WIDTH   = 12
)(
    // Write side domain
    input  wire                         i_wclk,
    input  wire                         i_wrstn,
    input  wire                         i_s_valid,
    output reg                          o_s_ready,
    input  wire  [DATA_WIDTH - 1 : 0]   i_s_data,
    input  wire                         i_s_last,
    
    // Read side domain
    input  wire                         i_rclk,
    input  wire                         i_rrstn,
    output wire                         o_m_valid,
    input  wire                         i_m_ready,
    output wire  [DATA_WIDTH - 1 : 0]   o_m_data,
    output wire                         o_m_last,
    input  wire                         i_swap_ok

);

    // ----------------------------------------------------------------
    // Derived constants
    // ----------------------------------------------------------------
    localparam FRAME_PIX        = FRAME_WIDTH * FRAME_HEIGHT;   // 76800
    localparam FRAME_ADDR_WIDTH = $clog2(FRAME_PIX);            // 17
    localparam MEM_DEPTH        = (1 << FRAME_ADDR_WIDTH);      // power of 2 >= FRAME_PIX
    
    // ================================================================
    // CDC HANDSHAKE SIGNALS (DO NOT DRIVE OUTSIDE THEIR DOMAIN)
    //   pub_tgl     : toggled ONLY in write clock domain
    //   ack_tgl     : toggled ONLY in read clock domain
    // ================================================================
    reg pub_tgl;
    reg ack_toggle;
    
    // ================================================================
    //                      WRITE SIDE FSM
    // ================================================================
    
    // state encoding
    reg [1:0] W_STATE, W_STATE_N;
    localparam [1:0] W_IDLE             = 2'b00,
                     W_FILL             = 2'b01,
                     W_WAIT_ACK         = 2'b10;
     
     // intermediate signals
     reg [FRAME_ADDR_WIDTH-1:0] wr_addr, wr_addr_n;
     reg wr_bank, wr_bank_n;
     reg pub_tgl_n;
     
     reg ack_w1, ack_w2;
     reg ack_seen, ack_seen_n;
     
     wire ack_event = (ack_w2 != ack_seen);
     wire publish_allowed = (pub_tgl == ack_seen);
     
     wire w_fire = i_s_valid && o_s_ready;
     
     // BRAM write enables (combinational outputs)
     reg wr_en0, wr_en1;
     
     always @ (*) begin
         // defaults: hold state
         W_STATE_N   = W_STATE;
         wr_addr_n   = wr_addr;
         wr_bank_n   = wr_bank;
         pub_tgl_n   = pub_tgl;
         ack_seen_n  = ack_seen;
     
         // defaults: outputs
         o_s_ready   = 1'b0;
         wr_en0      = 1'b0;
         wr_en1      = 1'b0;
     
         // consume ack event (purely local bookkeeping)
         if (ack_event)
           ack_seen_n = ack_w2;
     
         case (W_STATE)
             W_IDLE: begin
                 wr_addr_n = 1'b0;
                 o_s_ready = publish_allowed; // only start new frame if no outstanding publish
                 if (publish_allowed && i_s_valid) begin
                     W_STATE_N = W_FILL;
                 end
             end
     
             W_FILL: begin
                 o_s_ready = 1'b1;
     
                 if (w_fire) begin
                     // write this beat into current bank
                     if (wr_bank == 1'b0) begin 
                         wr_en0 = 1'b1;
                     end
                     else begin                
                         wr_en1 = 1'b1;
                     end
     
                     // advance write address
                     wr_addr_n = wr_addr + 1'b1;
     
                     // end-of-frame: publish and wait for ack
                     if (i_s_last) begin
                         pub_tgl_n = ~pub_tgl;
                         W_STATE_N = W_WAIT_ACK;
                         wr_addr_n = 1'b0;
                     end
                 end
             end
     
             W_WAIT_ACK: begin
                 o_s_ready = 1'b0; // stall producer
                 if (ack_event) begin
                     wr_bank_n = ~wr_bank; // next frame fills other bank
                     W_STATE_N = W_IDLE;
                 end
             end
     
             default: begin
                 W_STATE_N = W_IDLE;
             end
         endcase
     end
     
     always @(posedge i_wclk) begin
         if (!i_wrstn) begin
             W_STATE  <= W_IDLE;
             wr_addr  <= 1'b0;
             wr_bank  <= 1'b0;
             pub_tgl  <= 1'b0;
     
             ack_w1   <= 1'b0;
             ack_w2   <= 1'b0;
             ack_seen <= 1'b0;
         end 
         else begin
             // sync ack toggle into write domain
             ack_w1 <= ack_toggle;
             ack_w2 <= ack_w1;
     
             // register updates
             W_STATE  <= W_STATE_N;
             wr_addr  <= wr_addr_n;
             wr_bank  <= wr_bank_n;
             pub_tgl  <= pub_tgl_n;
             ack_seen <= ack_seen_n;
         end
     end 
    
    // ================================================================
    //                         READ SIDE FSM
    // ================================================================
    // state encoding
    reg [1:0] R_STATE, R_STATE_N;
    localparam [1:0] R_NOFRAME             = 2'b00,
                     R_PENDING             = 2'b01,
                     R_DRAIN               = 2'b10;

    reg rd_sel, rd_sel_n;
    reg [FRAME_ADDR_WIDTH-1:0] rd_addr, rd_addr_n;
    
    reg [FRAME_ADDR_WIDTH-1:0] rd_addr_q;

    reg pub_r1, pub_r2;
    reg pub_seen, pub_seen_n;
    wire  pub_event = (pub_r2 != pub_seen);
    
    reg pending, pending_n;
    
    reg ack_toggle_n;
    
    reg s_last_r1, s_last_r2;
    
    // streaming handshake
    wire r_fire = o_m_valid && i_m_ready;
    
    // one-cycle read latency pipeline regs
    reg m_valid_n, m_last_n;
    reg m_valid_q, m_last_q;
    reg rd_sel_q;  // bank select aligned to BRAM output
    
    localparam MEM_WORD_W = DATA_WIDTH + 1;

    wire [MEM_WORD_W-1:0] rword0, rword1;
    
    always @ (*) begin
        R_STATE_N     = R_STATE;
        rd_sel_n     = rd_sel;
        rd_addr_n     = rd_addr;
        pub_seen_n    = pub_seen;
        pending_n     = pending;
        ack_toggle_n  = ack_toggle;
    
        // defaults for output pipeline next
        m_valid_n     = 1'b0;
        m_last_n      = 1'b0;
    
        // latch pending when publish event arrives
        if (pub_event)
            pending_n = 1'b1;
    
        case (R_STATE)
            R_NOFRAME: begin
                rd_addr_n  = 1'b0;
                R_STATE_N = (pending_n) ? R_PENDING : R_NOFRAME;
            end
    
            R_PENDING: begin
                // swap only when policy allows
                if (pending && i_swap_ok) begin
                    rd_sel_n    = ~rd_sel;
                    pub_seen_n   = pub_r2;          // consume publish
                    pending_n    = 1'b0;
                    ack_toggle_n = ~ack_toggle;     // ack writer
                    rd_addr_n    = 1'b0;
                    R_STATE_N    = R_DRAIN;
                end
            end
    
          R_DRAIN: begin
              // We present valid continuously while draining;
              // actual acceptance advances on r_fire.
              m_valid_n = 1'b1;
              
              // synced i_s_last
              m_last_n  = s_last_r2; // (rd_addr == FRAME_PIX-1) for fixed length
    
              if (r_fire) begin
                  if (rd_addr == FRAME_PIX-1) begin
                      rd_addr_n = 1'b0;
                      R_STATE_N = R_NOFRAME;
                  end 
                  else begin
                      rd_addr_n = rd_addr + 1'b1;
                  end
              end
          end
    
          default: R_STATE_N = R_NOFRAME;
        endcase
    end
    
    always @(posedge i_rclk) begin
        if (!i_rrstn) begin
            R_STATE    <= R_NOFRAME;
            rd_sel     <= 1'b0;
            rd_addr    <= 1'b0;
            rd_addr_q  <= 1'b0;
            pub_r1     <= 1'b0;
            pub_r2     <= 1'b0;
            pub_seen   <= 1'b0;
            pending    <= 1'b0;
            ack_toggle <= 1'b0;
    
            // pipeline regs
            m_valid_q  <= 1'b0;
            m_last_q   <= 1'b0;
            rd_sel_q   <= 1'b0;
            
            
            s_last_r1  <= 1'b0;
            s_last_r2  <= 1'b0;
        end 
        else begin
            // sync publish toggle into read domain
            pub_r1 <= pub_tgl;
            pub_r2 <= pub_r1;
            
            // i_s_last write -> read 
            s_last_r1   <= i_s_last;
            s_last_r2   <= s_last_r1;
               
            // register updates
            R_STATE    <= R_STATE_N;
            rd_sel     <= rd_sel_n;
            rd_addr    <= rd_addr_n;
            rd_addr_q  <= rd_addr;
            pub_seen   <= pub_seen_n;
            pending    <= pending_n;
            ack_toggle <= ack_toggle_n;
    
            // pipeline alignment for synchronous-read BRAM
            m_valid_q  <= m_valid_n;
            rd_sel_q   <= rd_sel; 
            
            if (m_valid_n) begin
                m_last_q   <= (rd_sel_q==0) ? rword0[DATA_WIDTH] : rword1[DATA_WIDTH];
            end
            else begin
                m_last_q   <= 1'b0;
            end
        end
    end
    
    assign o_m_valid = m_valid_q;
    assign o_m_data = (rd_sel_q==0) ? rword0[DATA_WIDTH-1:0] : rword1[DATA_WIDTH-1:0];
    assign o_m_last = m_last_q; // where m_last_q comes from selected rword last-bit


    // ================================================================
    // BRAMs: one per buffer (unchanged structurally)
    // ================================================================
    wire [DATA_WIDTH-1:0] rdata0, rdata1;
    
    // buffer 0
    mem_bram #(
        .BRAM_WIDTH (DATA_WIDTH),
        .BRAM_DEPTH (MEM_DEPTH)     // power-of-2 depth
    ) mem0 (
        .i_wclk    (i_wclk),
        .i_wportEn (1'b1),
        .i_waddr   (wr_addr),
        .i_wdata   (i_s_data),
        .i_wr      (wr_en0),

        .i_rclk    (i_rclk),
        .i_rportEn (1'b1),
        .i_raddr   (rd_addr_q),
        .o_rdata   (rdata0)
    );

    // buffer 1
    mem_bram #(
        .BRAM_WIDTH (DATA_WIDTH),
        .BRAM_DEPTH (MEM_DEPTH)
    ) mem1 (
        .i_wclk    (i_wclk),
        .i_wportEn (1'b1),
        .i_waddr   (wr_addr),
        .i_wdata   (i_s_data),
        .i_wr      (wr_en1),

        .i_rclk    (i_rclk),
        .i_rportEn (1'b1),
        .i_raddr   (rd_addr_q),
        .o_rdata   (rdata1)
    );
    
    
    
    // =====================================================
    // Formal verification
    // =====================================================
    // wsl sby -f ping_pong_buf.sby 
    
    
    `ifdef FORMAL
    
        
      // real BRAMs
      mem_bram ... mem0(... .o_rdata(rdata0));
      mem_bram ... mem1(... .o_rdata(rdata1));
        
        
      // formal model
      reg [DATA_WIDTH-1:0] mem0 [0:MEM_DEPTH-1];
      reg [DATA_WIDTH-1:0] mem1 [0:MEM_DEPTH-1];
      reg [DATA_WIDTH-1:0] rdata0_f, rdata1_f;
    
      always @(posedge i_wclk) if (wr_en0) mem0[wr_addr] <= i_s_data;
      always @(posedge i_wclk) if (wr_en1) mem1[wr_addr] <= i_s_data;
    
      always @(posedge i_rclk) rdata0_f <= mem0[rd_addr];
      always @(posedge i_rclk) rdata1_f <= mem1[rd_addr];
    
      assign rdata0 = rdata0_f;
      assign rdata1 = rdata1_f;
        
        // ============================================
        //            Assumptions (Fairness)
        // ============================================
        
        // resets
        // Hold reset low for 1 cycle, then release forever (per domain)
        assume property (@(posedge i_wclk) $initstate |-> !i_wrstn);
        assume property (@(posedge i_rclk) $initstate |-> !i_rrstn);
        
        assume property (@(posedge i_wclk) !$initstate |-> i_wrstn);
        assume property (@(posedge i_rclk) !$initstate |-> i_rrstn);
        
        // tlast signal
        // comes only on an actual transfer
        assume property (@(posedge i_wclk)
            i_s_last |-> (i_s_valid && o_s_ready)
        );
        // data is stable while waiting
        assume property (@(posedge i_wclk)
            (i_s_valid && !o_s_ready) |-> $stable(i_s_data) && $stable(i_s_last)
        );
        // swap_ok eventually happens 
        assume property (@(posedge i_rclk) 1'b1 |-> ##[0:$] i_swap_ok);
        
        // downstream eventually accepts what DUT sends
        assume property (@(posedge i_rclk)
            o_m_valid |-> ##[0:$] i_m_ready
        );
        
        // writer stays in state W_FILL until tlast is sent (bounded)
        localparam int MAXLEN = MEM_DEPTH; // or FRAME_PIX for fixed video
        assume property (@(posedge i_wclk)
          (i_s_valid && o_s_ready) |-> ##[0:MAXLEN] (i_s_valid && o_s_ready && i_s_last)
        );
        
        // ===================================================
        //   Unbounded Assertions (eventual frame handoff)
        // ===================================================
        
        // when pub_tgl occurs
        wire pub_evt_w = (pub_tgl != $past(pub_tgl));
        
        // when ack occurs
        wire ack_evt_w = (ack_w2 != $past(ack_w2));
        
        // when pub_tgl occurs, ack should occur eventually
        assert property (@(posedge i_wclk)
          pub_evt_w |-> ##[0:$] ack_evt_w
        );
        
        // writer eventually switches frames
        assert property (@(posedge i_wclk)
          (W_STATE == W_WAIT_ACK) |-> ##[0:$] (W_STATE == W_IDLE)
        );
        
        // ===================================================
        //              Invariants (Safety)
        // ===================================================
        // *
        // *
        // * (Work in progress)
        // *
        // *
        
        
    `endif
    
   
endmodule

    

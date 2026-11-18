`timescale 1ns/1ps
module sota_liu_hybrid_mul(
 input wire signed [7:0] activation,
 input wire signed [3:0] weight,
 input wire [7:0] zero_threshold,
 input wire [7:0] four_threshold,
 input wire precision_enable,
 output wire signed [11:0] product,
 output wire [1:0] precision_tag
);
 wire neg=activation[7];
 wire [7:0] mag=neg ? (~activation+8'd1) : activation;
 wire is_zero=precision_enable && (mag<zero_threshold);
 wire is_four=precision_enable && !is_zero && (mag<four_threshold);
 assign precision_tag=is_zero ? 2'b00 : is_four ? 2'b01 : 2'b11;
 wire [4:0] q_hi=activation[7:3]+(is_four && neg && (|activation[2:0]));
 wire signed [7:0] q=is_zero ? 8'sd0 : {q_hi, is_four ? 3'b000 : activation[2:0]};
 assign product=q*weight;
endmodule

module sota_liu_full #(
 parameter FRAME_SAMPLES=400,
 parameter NUM_CLASSES=5
)(
 input wire clk, input wire reset, input wire start_cmd,
 input wire [1:0] op_mode,
 input wire sample_valid, input wire signed [7:0] sample_in,
 output wire sample_ready, output wire busy_status,
 input wire cfg_we, input wire [10:0] cfg_addr, input wire [7:0] cfg_data,
 input wire wake_enable, precision_enable, threshold_update_enable,
 input wire [7:0] initial_threshold,
 output reg done_irq, output reg [2:0] pred_class,
 output reg signed [23:0] pred_score,
 output wire [NUM_CLASSES*24-1:0] scores_out,
 output reg nn_executed, output reg forced_check,
 output reg [7:0] threshold_out,
 output wire [15:0] zero_lanes, four_lanes, eight_lanes
);
 localparam IDLE=0, CAPTURE=1, CHECK=2, MAC=3, DECIDE=4, DRAIN=5;
 reg [2:0] state;
 reg [3:0] pipe_valid, pipe_last;
 reg [8:0] sample_index;
 reg [4:0] group_index;
 reg [2:0] group_mod5;
 reg [2:0] class_index;
 reg [4:0] consecutive_bypass;
 reg [8:0] l1_rem;
 reg [7:0] metric;
 reg signed [23:0] accumulator;
 reg signed [23:0] scores[0:NUM_CLASSES-1];
 reg signed [23:0] best, second;
 reg [2:0] winner;
 reg [1:0] active_mode;

 reg signed [7:0] template_mem[0:FRAME_SAMPLES-1];
 reg signed [7:0] template_value;
 always @(*) begin
   case(sample_index[3:0])
     4'd0: template_value = template_mem[0];
     4'd1: template_value = template_mem[1];
     4'd2: template_value = template_mem[2];
     4'd3: template_value = template_mem[3];
     4'd4: template_value = template_mem[4];
     4'd5: template_value = template_mem[5];
     4'd6: template_value = template_mem[6];
     4'd7: template_value = template_mem[7];
     4'd8: template_value = template_mem[8];
     4'd9: template_value = template_mem[9];
     4'd10: template_value = template_mem[10];
     4'd11: template_value = template_mem[11];
     4'd12: template_value = template_mem[12];
     4'd13: template_value = template_mem[13];
     4'd14: template_value = template_mem[14];
     default: template_value = template_mem[15];
   endcase
 end

 wire signed [8:0] difference_wide = $signed({sample_in[7],sample_in}) - $signed({template_value[7],template_value});
 wire signed [7:0] difference = difference_wide>127 ? 8'sd127 : difference_wide< -128 ? -8'sd128 : difference_wide[7:0];
 wire [7:0] difference_abs = difference[7] ? (~difference+8'd1) : difference;
 wire [9:0] rem_sum = {1'b0, l1_rem} + {2'd0, difference_abs};
 wire rem_ge = rem_sum >= 10'd400;
 wire [8:0] rem_next = rem_ge ? (rem_sum - 10'd400) : rem_sum[8:0];

 wire capturing = state==CAPTURE && sample_valid;
 wire computing = state==MAC;
 assign sample_ready = state==CAPTURE;
 assign busy_status = state!=IDLE;
 wire signed [11:0] lane_product[0:15];
 wire [1:0] lane_tag[0:15];

 genvar lane;
 generate for(lane=0;lane<16;lane=lane+1)begin: lanes
   reg signed [7:0] diff_bank[0:(FRAME_SAMPLES/16)-1];
   reg signed [3:0] weights[0:NUM_CLASSES-1];
   reg signed [7:0] operand;
   reg signed [3:0] weight_value;
   integer r;
   always @(posedge clk) begin
     if(reset) begin
       operand<=0;weight_value<=0;
     end else if(computing) begin
       case(group_mod5)
         3'd0: operand<=diff_bank[0];
         3'd1: operand<=diff_bank[1];
         3'd2: operand<=diff_bank[2];
         3'd3: operand<=diff_bank[3];
         default: operand<=diff_bank[4];
       endcase
       case(class_index)
         3'd0: weight_value<=weights[0];
         3'd1: weight_value<=weights[1];
         3'd2: weight_value<=weights[2];
         3'd3: weight_value<=weights[3];
         default: weight_value<=weights[4];
       endcase
     end
   end

   always @(posedge clk) begin
     for(r=0; r<(FRAME_SAMPLES/16); r=r+1) begin
       if((capturing && sample_index[3:0]==lane && (r%5)==group_mod5) || (computing && group_mod5==3'd4)) begin
         diff_bank[r] <= (r < (FRAME_SAMPLES/16)-5) ? diff_bank[r+5] : (computing ? diff_bank[r-20] : difference);
       end
     end
     if(cfg_we && state==IDLE && cfg_addr>=512 && cfg_addr<512+NUM_CLASSES*16 && cfg_addr[3:0]==lane)
       weights[cfg_addr[6:4]] <= cfg_data[3:0];
   end

   sota_liu_hybrid_mul mul(.activation(operand),.weight(weight_value),
      .zero_threshold(8'd8),.four_threshold(8'd56),.precision_enable(precision_enable),
      .product(lane_product[lane]),.precision_tag(lane_tag[lane]));
   assign zero_lanes[lane]=pipe_valid[0] && lane_tag[lane]==0;
   assign four_lanes[lane]=pipe_valid[0] && lane_tag[lane]==1;
   assign eight_lanes[lane]=pipe_valid[0] && lane_tag[lane]==3;
 end endgenerate

 generate for(lane=0;lane<NUM_CLASSES;lane=lane+1)begin: score_ports
   assign scores_out[lane*24+:24]=scores[lane];
 end endgenerate

 integer i,t_idx;
 reg signed [11:0] product_pipe[0:15];
 reg signed [13:0] sum4_pipe[0:3];
 reg signed [15:0] sum16_pipe;
 wire signed [12:0] sum2[0:7];
 wire signed [13:0] sum4[0:3];
 wire signed [14:0] sum8[0:1];
 wire signed [15:0] sum16;
 wire signed [23:0] group_sum={{8{sum16_pipe[15]}},sum16_pipe};
 wire signed [23:0] next_acc = accumulator + group_sum;

 genvar n;
 generate
  for(n=0;n<8;n=n+1)begin: tree2
   assign sum2[n]=$signed({product_pipe[2*n][11],product_pipe[2*n]})+$signed({product_pipe[2*n+1][11],product_pipe[2*n+1]});
  end
  for(n=0;n<4;n=n+1)begin: tree4
   assign sum4[n]=$signed({sum2[2*n][12],sum2[2*n]})+$signed({sum2[2*n+1][12],sum2[2*n+1]});
  end
  for(n=0;n<2;n=n+1)begin: tree8
   assign sum8[n]=$signed({sum4_pipe[2*n][13],sum4_pipe[2*n]})+$signed({sum4_pipe[2*n+1][13],sum4_pipe[2*n+1]});
  end
 endgenerate
 assign sum16=$signed({sum8[0][14],sum8[0]})+$signed({sum8[1][14],sum8[1]});

 wire template_config = cfg_we && state==IDLE && cfg_addr<FRAME_SAMPLES;
 wire template_update = capturing && active_mode==2;
 wire signed [8:0] template_step = $signed(difference_wide)>>>5;
 wire [7:0] template_next = template_value + template_step[7:0];

 always @(posedge clk) begin
   if(template_config || capturing) begin
     for(t_idx=0; t_idx<FRAME_SAMPLES; t_idx=t_idx+1) begin
       if(t_idx[3:0] == (template_config ? cfg_addr[3:0] : sample_index[3:0])) begin
         if(t_idx < FRAME_SAMPLES-16)
           template_mem[t_idx] <= template_mem[t_idx+16];
         else
           template_mem[t_idx] <= template_config ? cfg_data : (template_update ? template_next : template_value);
       end
     end
   end

   if(reset)begin
     pipe_valid<=0;pipe_last<=0;
     state<=IDLE;sample_index<=0;group_index<=0;group_mod5<=0;class_index<=0;consecutive_bypass<=0;
     l1_rem<=0;metric<=0;accumulator<=0;active_mode<=0;done_irq<=0;pred_class<=0;pred_score<=0;
     nn_executed<=0;forced_check<=0;threshold_out<=initial_threshold;
     best<=0;second<=0;winner<=0;
     for(i=0;i<NUM_CLASSES;i=i+1)scores[i]<=0;
   end else begin
     pipe_valid<={pipe_valid[2:0],computing};
     pipe_last<={pipe_last[2:0],computing && group_index==5'd24};
     if(pipe_valid[0])for(i=0;i<16;i=i+1)product_pipe[i]<=lane_product[i];
     if(pipe_valid[1])for(i=0;i<4;i=i+1)sum4_pipe[i]<=sum4[i];
     if(pipe_valid[2])sum16_pipe<=sum16;
     done_irq<=0;
     case(state)
       IDLE:if(start_cmd)begin
         state<=CAPTURE;sample_index<=0;group_mod5<=0;l1_rem<=0;metric<=0;active_mode<=op_mode;
         nn_executed<=0;forced_check<=0;
       end
       CAPTURE:if(sample_valid)begin
         l1_rem<=rem_next;
         if(rem_ge)metric<=metric+8'd1;
         if(sample_index==FRAME_SAMPLES-1)begin state<=CHECK;group_mod5<=0;end
         else begin
           sample_index<=sample_index+1'b1;
           if(sample_index[3:0]==4'd15) group_mod5<=(group_mod5==3'd4)?3'd0:(group_mod5+3'd1);
         end
       end
       CHECK:begin
         if(active_mode==2)begin done_irq<=1;state<=IDLE;end
         else if(wake_enable && metric<threshold_out && consecutive_bypass<20)begin
           pred_class<=0;pred_score<=0;done_irq<=1;state<=IDLE;
           consecutive_bypass<=consecutive_bypass+1'b1;
           for(i=0;i<NUM_CLASSES;i=i+1)scores[i]<=0;
         end else begin
           nn_executed<=1;forced_check<=wake_enable && consecutive_bypass>=20;
           consecutive_bypass<=0;group_index<=0;group_mod5<=0;class_index<=0;accumulator<=0;
           for(i=0;i<NUM_CLASSES;i=i+1)scores[i]<=0;
           state<=MAC;
         end
       end
       MAC:begin
         if(group_index==5'd24)state<=DRAIN;
         else begin group_index<=group_index+1'b1; group_mod5<=(group_mod5==3'd4)?3'd0:(group_mod5+3'd1); end
       end
       DRAIN:begin end
       DECIDE:begin
         pred_class<=winner;pred_score<=best;done_irq<=1;state<=IDLE;
         if(threshold_update_enable && best-second>=64)begin
           if(winner==0)threshold_out<=((threshold_out*3)+metric)>>2;
           else threshold_out<=metric>1 ? metric>>1 : 8'd1;
         end
       end
       default:state<=IDLE;
     endcase
     if(pipe_valid[3])begin
       if(pipe_last[3])begin
         scores[class_index]<=next_acc;
         if(class_index==0)begin
           best<=next_acc;second<=-24'sd8388607;winner<=0;
         end else if(next_acc>best)begin
           second<=best;best<=next_acc;winner<=class_index;
         end else if(next_acc>second)begin
           second<=next_acc;
         end
         accumulator<=0;group_index<=0;group_mod5<=0;
         if(class_index==NUM_CLASSES-1)state<=DECIDE;
         else begin class_index<=class_index+1'b1;state<=MAC;end
       end else accumulator<=next_acc;
     end
   end
 end
endmodule


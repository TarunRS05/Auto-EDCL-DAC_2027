`timescale 1ns/1ps

module tb_sota_liu_full;
 reg clk=0,reset=1,start_cmd=0,sample_valid=0,cfg_we=0;
 reg [1:0] op_mode=1;
 reg signed [7:0] sample_in=0;
 reg [10:0] cfg_addr=0;
 reg [7:0] cfg_data=0;
 wire sample_ready,busy_status,done_irq,nn_executed,forced_check;
 wire [2:0] pred_class;
 wire signed [23:0] pred_score;
 wire [119:0] scores_out;
 wire [7:0] threshold_out;
 wire [15:0] zero_lanes,four_lanes,eight_lanes;
 sota_liu_full dut(
  .clk(clk),.reset(reset),.start_cmd(start_cmd),.op_mode(op_mode),
  .sample_valid(sample_valid),.sample_in(sample_in),.sample_ready(sample_ready),
  .busy_status(busy_status),.cfg_we(cfg_we),.cfg_addr(cfg_addr),.cfg_data(cfg_data),
  .wake_enable(1'b1),.precision_enable(1'b1),.threshold_update_enable(1'b1),
  .initial_threshold(8'd12),.done_irq(done_irq),.pred_class(pred_class),
  .pred_score(pred_score),.scores_out(scores_out),.nn_executed(nn_executed),
  .forced_check(forced_check),.threshold_out(threshold_out),
  .zero_lanes(zero_lanes),.four_lanes(four_lanes),.eight_lanes(eight_lanes));
 always #0.5 clk=~clk;

 reg signed [7:0] samples[0:2047];
 reg [7:0] weight_bytes[0:63];
 reg signed [7:0] reference_template[0:511];
 reg signed [3:0] reference_weights[0:79];
 integer reference_scores[0:4];
 integer reference_threshold=12,reference_bypasses=0;
 integer reference_index=0,reference_l1=0,reference_mode=0;
 integer accepted=0,completed=0,enrollments=0,inferences=0,wakes=0,bypasses=0,forced=0;
 integer threshold_changes=0,benchmark_edges=0,command_index=0;
 integer cycle,i,c,wide,difference,magnitude,quantized,metric,best,second,winner,next_threshold;
 reg benchmark_active=0;
 integer sf;
 reg [1023:0] vcd_path,stats_path;

 integer dump_word;
 initial begin
  #0;
  for(dump_word=0;dump_word<400;dump_word=dump_word+1)
   $dumpvars(0,dut.template_mem[dump_word]);
 end
 genvar dump_lane;
 generate for(dump_lane=0;dump_lane<16;dump_lane=dump_lane+1)begin: dump_memories
  integer word_index;
  initial begin
   #0;
   for(word_index=0;word_index<25;word_index=word_index+1)
    $dumpvars(0,dut.lanes[dump_lane].diff_bank[word_index]);
   for(word_index=0;word_index<5;word_index=word_index+1)
    $dumpvars(0,dut.lanes[dump_lane].weights[word_index]);
  end
 end endgenerate

 always @(posedge clk)if(benchmark_active)begin
  benchmark_edges=benchmark_edges+1;
  if(start_cmd && !busy_status)begin
   reference_mode=op_mode;reference_index=0;reference_l1=0;
   for(c=0;c<5;c=c+1)reference_scores[c]=0;
  end
  if(sample_valid && sample_ready)begin
   wide=$signed(sample_in)-$signed(reference_template[reference_index]);
   difference=wide>127 ? 127 : wide< -128 ? -128 : wide;
   magnitude=difference<0 ? -difference : difference;
   reference_l1=reference_l1+magnitude;
   quantized=magnitude<8 ? 0 : magnitude<56 ?
      (difference<0 ? -1 : 1)*((magnitude/8)*8) : difference;
   for(c=0;c<5;c=c+1)
    reference_scores[c]=reference_scores[c]+quantized*$signed(reference_weights[c*16+(reference_index%16)]);
   if(reference_mode==2)
    reference_template[reference_index]=reference_template[reference_index]+(wide>>>5);
   reference_index=reference_index+1;accepted=accepted+1;
  end
  if(done_irq)begin
   if(reference_index!=400)$fatal(1,"Incomplete frame at done: %0d",reference_index);
   completed=completed+1;
   if(reference_mode==2)begin
    enrollments=enrollments+1;
    if(nn_executed || forced_check)$fatal(1,"Enrollment incorrectly executes NN");
   end else begin
    inferences=inferences+1;metric=reference_l1/400;
    if(metric<reference_threshold && reference_bypasses<20)begin
     bypasses=bypasses+1;reference_bypasses=reference_bypasses+1;
     if(nn_executed || forced_check || pred_class!==0 || pred_score!==0 || scores_out!==0)
      $fatal(1,"Wake bypass mismatch");
    end else begin
     wakes=wakes+1;
     if(!nn_executed || forced_check!==(reference_bypasses>=20))$fatal(1,"Wake/forced flag mismatch");
     if(forced_check)forced=forced+1;
     reference_bypasses=0;best=-8388607;second=-8388607;winner=0;
     for(c=0;c<5;c=c+1)begin
      if($signed(scores_out[c*24+:24])!==reference_scores[c])
       $fatal(1,"Score mismatch class=%0d got=%0d expected=%0d",c,$signed(scores_out[c*24+:24]),reference_scores[c]);
      if(reference_scores[c]>best)begin second=best;best=reference_scores[c];winner=c;end
      else if(reference_scores[c]>second)second=reference_scores[c];
     end
     if(pred_class!==winner || $signed(pred_score)!==best)$fatal(1,"Winner mismatch");
     if(best-second>=64)begin
      next_threshold=winner==0 ? ((reference_threshold*3)+metric)>>2 : (metric>1 ? metric>>1 : 1);
      if(next_threshold!=reference_threshold)threshold_changes=threshold_changes+1;
      reference_threshold=next_threshold;
     end
    end
   end
   if(threshold_out!==reference_threshold)$fatal(1,"Calibration threshold mismatch");
  end
 end

 initial begin
  if(!$value$plusargs("VCD=%s",vcd_path))vcd_path="vcd/sota_liu_full/sota_liu_full.vcd";
  if(!$value$plusargs("STATS=%s",stats_path))stats_path="sim/build/sota_liu_full_stats.txt";
  $readmemh("sim/test_vectors/ecg_mitbih_raw_stream.hex",samples);
  $readmemh("sim/test_vectors/ecg_mitbih_weights.hex",weight_bytes);
  for(i=0;i<2048;i=i+1)if(^samples[i]===1'bx)$fatal(1,"Invalid ECG sample %0d",i);
  for(i=0;i<64;i=i+1)if(^weight_bytes[i]===1'bx)$fatal(1,"Invalid weight %0d",i);
  $dumpfile(vcd_path);$dumpvars(0,dut);
  repeat(50)@(negedge clk);
  reset=0;
  for(i=0;i<512;i=i+1)begin
   reference_template[i]=0;cfg_we=1;cfg_addr=i;cfg_data=0;@(negedge clk);
  end
  for(i=0;i<80;i=i+1)begin
   reference_weights[i]=weight_bytes[i%64][3:0];
   cfg_we=1;cfg_addr=512+i;cfg_data=weight_bytes[i%64];@(negedge clk);
  end
  cfg_we=0;benchmark_active=1;
  for(cycle=0;cycle<100000;cycle=cycle+1)begin
   start_cmd=0;sample_valid=0;
   if(!busy_status && !done_irq)begin
    op_mode=(command_index%8==0) ? 2 : 1;
    command_index=command_index+1;start_cmd=1;
   end
   if(sample_ready)begin sample_valid=1;sample_in=samples[accepted%2048];end
   @(negedge clk);
  end
  benchmark_active=0;start_cmd=0;sample_valid=0;
  if(benchmark_edges!=100000 || accepted<10000 || enrollments==0 || wakes==0)
   $fatal(1,"Insufficient benchmark activity");
  sf=$fopen(stats_path,"w");if(!sf)$fatal(1,"Cannot open stats output");
  $fdisplay(sf,"benchmark_cycles=%0d",benchmark_edges);
  $fdisplay(sf,"accepted_samples=%0d",accepted);
  $fdisplay(sf,"completed_commands=%0d",completed);
  $fdisplay(sf,"inference_commands=%0d",inferences);
  $fdisplay(sf,"template_enrollments=%0d",enrollments);
  $fdisplay(sf,"nn_executed=%0d",wakes);
  $fdisplay(sf,"bypassed_frames=%0d",bypasses);
  $fdisplay(sf,"forced_check_frames=%0d",forced);
  $fdisplay(sf,"threshold_changes=%0d",threshold_changes);
  $fdisplay(sf,"end_time_ps=%0d",$time*1000);
  $fclose(sf);
  $display("PASS: tb_sota_liu_full 100000-cycle benchmark, %0d completed, %0d NN, %0d EMA",completed,wakes,enrollments);
  $finish;
 end
 initial begin #110000;$fatal(1,"Benchmark timeout");end
endmodule


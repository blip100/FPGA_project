module tb();
    reg fast_clk,man_clk;
    always #0.5 fast_clk = ~fast_clk;

    top_module T0(fast_clk,man_clk);
    initial begin
        fast_clk = 0;
        man_clk = 1;
        #10000000000 $finish;
    end

endmodule
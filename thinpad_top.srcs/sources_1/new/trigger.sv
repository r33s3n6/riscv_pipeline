`default_nettype none

module trigger(
    input wire clk,
    input wire rst,
    input wire btn,
    output wire trigger
);

logic prev_btn_reg;
logic trigger_reg;

assign trigger = trigger_reg;

always_ff @(posedge clk) begin
    if (rst) begin
        trigger_reg <= 1'b0;
        prev_btn_reg <= 1'b0;
    end else begin
        prev_btn_reg <= btn;
        if (btn && !prev_btn_reg) begin
            trigger_reg <= 1'b1;
        end else begin
            trigger_reg <= 1'b0;
        end
    end

end


endmodule


`default_nettype none

module counter(
    input wire clk,
    input wire rst,
    input wire trigger,
    output wire [3:0] count
);

    logic [3:0] count_reg;
    assign count = count_reg;

    always_ff @(posedge trigger or posedge rst) begin
        if (rst) begin
            count_reg <= 4'b0000;
        end else begin
            if(count_reg == 4'b1111) begin
                count_reg <= 4'b1111;
            end else begin
                count_reg <= count_reg + 1;
            end
        end
    end


endmodule
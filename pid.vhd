library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity pid is
	port (
		RESET:    in  std_logic;
		CLK:      in  std_logic;
		
		START:    in  std_logic;
		
		VAL_IN:   in  signed(15 downto 0);
		SETPNT:   in  signed(15 downto 0);
		
		P0:       in  signed(31 downto 0); -- Ki*T
		P1:       in  signed(31 downto 0); -- -Kp - Kd/T
		P2:       in  signed(31 downto 0); -- Kp + 2Kd/T
		P3:       in  signed(31 downto 0); -- -Kd/T
		
		CTRL_OUT: out signed(15 downto 0)
	);
end entity pid;

architecture rtl of pid is
	
	component mult32 is
		port
		(
			clock0		: in std_logic  := '1';
			dataa_0		: in std_logic_vector (31 downto 0) :=  (others => '0');
			datab_0		: in std_logic_vector (31 downto 0) :=  (others => '0');
			result		: out std_logic_vector (63 downto 0)
		);
	end component mult32;
	
	constant PIPELINE_LEN: natural := 4;

	signal coeff: signed(31 downto 0);
	signal term:  signed(31 downto 0);
	
	type   val_buf_t is array(0 to 2) of signed(15 downto 0);
	signal val_buf: val_buf_t;
	
	signal pipeline_cnt: integer range 0 to PIPELINE_LEN+4;
	signal accu: signed(31 downto 0) := (others => '0');
	signal mul_result_slv: std_logic_vector(31 downto 0);
	signal mul_result: signed(63 downto 0);
	signal running: std_logic;
	signal done: std_logic;
begin
	
	mult_inst: mult32 port map (
		clock0 => CLK,
		dataa_0 => std_logic_vector(term),
		datab_0 => std_logic_vector(coeff),
		result => mul_result_slv);
		
	mul_result <= signed(mul_result_slv);
	
	pipeline_proc: process(CLK, RESET)
	begin
		if RESET = '1' then
			pipeline_cnt <= 0;
		elsif rising_edge(CLK) then
			if running = '0' then
				done <= '0';
				if START = '1' then
					running <= '1';
					pipeline_cnt <= pipeline_cnt + 1;
				end if;
			else
				if pipeline_cnt = PIPELINE_LEN+4 then
					done <= '1';
					if START = '0' then
						running <= '0';
						pipeline_cnt <= 0;
					end if;
				else
					pipeline_cnt <= pipeline_cnt + 1;
				end if;
			end if;
		end if;
	end process;
	
	mult_pl: process(CLK)
	begin
		if rising_edge(CLK) then
			case pipeline_cnt is
				when 1 =>
					coeff <= P1;
					term <= resize(val_buf(0), term'LENGTH);
				when 2 =>
					coeff <= P2;
					term <= resize(val_buf(1), term'LENGTH);
				when 3 =>
					coeff <= P3;
					term <= resize(val_buf(2), term'LENGTH);
				when others => -- default including 0
					coeff <= P0;
					term <= resize(SETPNT - VAL_IN, term'LENGTH); -- error
			end case;
		end if;
	end process;
	
	accu_pl: process(CLK)
	begin
		if rising_edge(CLK) then
			if (pipeline_cnt = PIPELINE_LEN+0 or
			    pipeline_cnt = PIPELINE_LEN+1 or
			    pipeline_cnt = PIPELINE_LEN+2 or
			    pipeline_cnt = PIPELINE_LEN+3)
			then
				accu <= accu + resize(mul_result, 32);
			end if;		
		end if;
	end process;
	
	

end architecture rtl;

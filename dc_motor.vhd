library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity dc_motor is
	port (
		CLK: in std_logic;

		SPEED: in std_logic_vector(15 downto 0);
		BREAK: in std_logic;

		BRIDGE1: out std_logic;
		BRIDGE2: out std_logic);
end entity dc_motor;

architecture rtl of dc_motor is
	signal pwm_cnt: unsigned(15 downto 0) := (others => '0');
	signal speed_buf: unsigned(14 downto 0);
	signal break_buf: std_logic;

begin

	cnt_proc: process(CLK)
	begin
		if rising_edge(CLK) then
			pwm_cnt <= pwm_cnt + 1;
		end if;
	end process;

	pwm_proc: process(CLK)
	begin
		if pwm_cnt = 0 then
			if SPEED(15) = '0' then
				speed_buf <= unsigned(SPEED(14 downto 0));
				if SPEED(14 downto 0) /= 0 then
					BRIDGE1 <= '1';
					BRIDGE2 <= '0';
				else
					BRIDGE1 <= BREAK;
					BRIDGE2 <= BREAK;
				end if;
			else
				speed_buf <= unsigned(not SPEED(14 downto 0)) + 1;
				BRIDGE1 <= '0';
				BRIDGE2 <= '1';
			end if;
			break_buf <= BREAK;
		elsif pwm_cnt = speed_buf then
			BRIDGE1 <= break_buf;
			BRIDGE2 <= break_buf;
		end if;
	end process;

end architecture rtl;

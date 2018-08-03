library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity reg_if is
	port (
		CLK: in std_logic;
		RST: in std_logic;

		SERIAL_IN:  in  std_logic;
		SERIAL_OUT: out std_logic;

		SPEED1: in  signed(15 downto 0);
		SPEED2: in  signed(15 downto 0);

		POS1: in  signed(31 downto 0);
		POS2: in  signed(31 downto 0);

		SETPNT1: out signed(15 downto 0);
		SETPNT2: out signed(15 downto 0);

		P0: out signed(31 downto 0);
		P1: out signed(31 downto 0);
		P2: out signed(31 downto 0);
		P3: out signed(31 downto 0);

		RESET_POS: out signed(31 downto 0);
		RESET_CMD1: out std_logic;
		RESET_CMD2: out std_logic
	);
end entity reg_if;

architecture rtl of reg_if is
	component uart
		port (
		  CLK        : in  std_logic;
		  RST        : in  std_logic;
		  TX_FULL    : out std_logic;
		  TX_DATA    : in  std_logic_vector(7 downto 0);
		  TX_WRITE   : in  std_logic;
		  RX_EMPTY   : out std_logic;
		  RX_DATA    : out std_logic_vector(7 downto 0);
		  RX_READ    : in  std_logic;
		  SERIAL_IN  : in  std_logic;
		  SERIAL_OUT : out std_logic
		);
	end component uart;

	signal rx_data     : std_logic_vector(7 downto 0);
	signal tx_data     : std_logic_vector(7 downto 0);

	signal tx_full     : std_logic;
	signal rx_empty    : std_logic;

	signal tx_write    : std_logic;
	signal rx_read     : std_logic;

	signal tx_buf      : std_logic_vector(31 downto 0);
	signal rx_buf      : std_logic_vector(23 downto 0);

	signal tx_cnt      : integer range 0 to 4;
	signal rx_cnt      : integer range 0 to 4;

	signal cmd         : std_logic_vector(7 downto 0);
begin

	uart_i : uart
		port map (
		  CLK        => CLK,
		  RST        => RST,
		  TX_FULL    => tx_full,
		  TX_DATA    => tx_data,
		  TX_WRITE   => tx_write,
		  RX_EMPTY   => rx_empty,
		  RX_DATA    => rx_data,
		  RX_READ    => rx_read,
		  SERIAL_IN  => SERIAL_IN,
		  SERIAL_OUT => SERIAL_OUT
		);

		tx_data <= tx_buf(31 downto 24);
		tx_write <= '1' when tx_cnt /= 0 else '0';
		rx_read <= '1' when rx_empty = '0' else '0';

	reg_proc: process(RST, CLK)
	begin
		if RST = '1' then
			SETPNT1 <= (others => '0');
			SETPNT2 <= (others => '0');

			P0 <= (others => '0');
			P1 <= (others => '0');
			P3 <= (others => '0');
			P2 <= (others => '0');

		elsif rising_edge(CLK) then
			RESET_CMD1 <= '0';
			RESET_CMD2 <= '0';

			if rx_read = '1' then
				if rx_cnt = 0 then -- expecting command
					cmd <= rx_data;

					if rx_data(7) = '1' and tx_write = '0' then
						-- read register
						case rx_data(2 downto 0) is
							when "000" =>
								tx_buf <= std_logic_vector(SPEED2 & SPEED1);
							when "010" =>
								tx_buf <= std_logic_vector(POS1);
							when "011" =>
								tx_buf <= std_logic_vector(POS2);
							when others =>
								tx_buf <= x"deadbeef";
						end case;
						tx_cnt <= 4;
					else
						-- write ...
						if rx_data(6) = '1' then
							-- command
							case rx_data(2 downto 0) is
								when "000" =>
									RESET_CMD1 <= '1';
								when "001" =>
									RESET_CMD2 <= '1';
								when others =>
							end case;
						else
							-- register
							rx_cnt <= 4;
						end if;
					end if;
				else -- expecting data
					rx_cnt <= rx_cnt - 1;
					rx_buf <= rx_buf(15 downto 0) & rx_data;
					if rx_cnt = 1 then
						-- received last data Word
						case cmd(2 downto 0) is
							when "000" => --SETPOINT
								SETPNT1 <= signed(rx_buf(7  downto 0) & rx_data);
								SETPNT2 <= signed(rx_buf(23 downto 8));
							when "001" => --RESET POS
								RESET_POS <= signed(rx_buf & rx_data);
							when "100" => --P0
								P0 <= signed(rx_buf & rx_data);
							when "101" => --P1
								P1 <= signed(rx_buf & rx_data);
							when "110" => --P2
								P2 <= signed(rx_buf & rx_data);
							when "111" => --P3
								P3 <= signed(rx_buf & rx_data);
							when others =>
						end case;
					end if;
				end if;
			end if;
			if tx_full = '0' and tx_write /= '1' then
				tx_cnt <= tx_cnt - 1;
			end if;
		end if;
	end process;


end architecture rtl;

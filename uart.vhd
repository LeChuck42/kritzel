library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

entity uart is
	port (
		CLK: in  std_logic;
		RST: in  std_logic;

		TX_FULL: out std_logic;
		TX_DATA: in  std_logic_vector(7 downto 0);
		TX_WRITE: in  std_logic;

		RX_EMPTY: out std_logic;
		RX_DATA: out std_logic_vector(7 downto 0);
		RX_READ: in  std_logic;

		SERIAL_IN: in  std_logic;
		SERIAL_OUT: out std_logic);
end entity;

architecture rtl of uart is
	component uart_transmitter is
		port (
			CLK         : in std_logic;                             -- Clock
			RST         : in std_logic;                             -- Reset
			TXCLK       : in std_logic;                             -- Transmitter clock (2x baudrate)
			TXSTART     : in std_logic;                             -- Start transmitter
			CLEAR       : in std_logic;                             -- Clear transmitter state
			WLS         : in std_logic_vector(1 downto 0);          -- Word length select
			STB         : in std_logic;                             -- Number of stop bits
			PEN         : in std_logic;                             -- Parity enable
			EPS         : in std_logic;                             -- Even parity select
			SP          : in std_logic;                             -- Stick parity
			BC          : in std_logic;                             -- Break control
			DIN         : in std_logic_vector(7 downto 0);          -- Input data
			TXFINISHED  : out std_logic;                            -- Transmitter operation finished
			SOUT        : out std_logic                             -- Transmitter output
		);
	end component uart_transmitter;

	component uart_receiver is
		port (
			CLK         : in std_logic;                             -- Clock
			RST         : in std_logic;                             -- Reset
			RXCLK       : in std_logic;                             -- Receiver clock (16x baudrate)
			RXCLEAR     : in std_logic;                             -- Reset receiver state
			WLS         : in std_logic_vector(1 downto 0);          -- Word length select
			STB         : in std_logic;                             -- Number of stop bits
			PEN         : in std_logic;                             -- Parity enable
			EPS         : in std_logic;                             -- Even parity select
			SP          : in std_logic;                             -- Stick parity
			SIN         : in std_logic;                             -- Receiver input
			PE          : out std_logic;                            -- Parity error
			FE          : out std_logic;                            -- Framing error
			BI          : out std_logic;                            -- Break interrupt
			DOUT        : out std_logic_vector(7 downto 0);         -- Output data
			RXFINISHED  : out std_logic                             -- Receiver operation finished
		);
	end component uart_receiver;

	signal rx_buf      : std_logic_vector(7 downto 0);
	signal tx_buf      : std_logic_vector(7 downto 0);

	signal dout        : std_logic_vector(7 downto 0);
	signal din         : std_logic_vector(7 downto 0);

	signal tx_finished : std_logic;
	signal rx_finished : std_logic;

	signal tx_buf_full : std_logic;
	signal tx_start    : std_logic;
	signal tx_busy     : std_logic;
	signal tx_ready    : std_logic;
	
	signal clk_div_cnt : unsigned(2 downto 0);
	signal clk_en_2    : std_logic;
begin
	uart_rx: uart_receiver port map (
		CLK        => CLK,
		RST        => RST,
		RXCLK      => '1',
		RXCLEAR    => '0',
		WLS        => "11",
		STB        => '0',
		PEN        => '1',
		EPS        => '1',
		SP         => '0',
		SIN        => SERIAL_IN,
		PE         => open,
		FE         => open,
		BI         => open,
		DOUT       => rx_buf,
		RXFINISHED => rx_finished);

	uart_tx: uart_transmitter port map (
		CLK        => CLK,
		RST        => RST,
		TXCLK      => clk_en_2,
		TXSTART    => tx_start,
		CLEAR      => '0',
		WLS        => "11", -- 8 Bit
		STB        => '0',  -- 1 Stop bit
		PEN        => '1',  -- Parity enable
		EPS        => '1',  -- Even parity
		SP         => '0',  -- No Sticky Parity
		BC         => '0',  -- Always enable
		DIN        => tx_buf,
		TXFINISHED => tx_finished,
		SOUT       => SERIAL_OUT);

	clk_div_proc: process(CLK)
	begin
		if rising_edge(CLK) then
			clk_div_cnt <= clk_div_cnt + 1;
			if clk_div_cnt = "000" then
				clk_en_2 <= '1';
			else
				clk_en_2 <= '0';
			end if;
		end if;
	end process;

	TX_FULL <= not tx_ready;
	tx_ready <= '1' when tx_busy = '0' or tx_finished = '1' else '0';
	tx_start <= TX_WRITE;

	tx_buf_proc: process(CLK, RST)
	begin
		if RST = '1' then
			tx_buf_full <= '0';
		elsif rising_edge(CLK) then

			if tx_finished = '1' then
				tx_busy <= '0';
			end if;

			if TX_WRITE = '1' and tx_ready = '1' then
				tx_busy <= '1';
				tx_buf <= TX_DATA;
			end if;

		end if;
	end process;

	rx_buf_proc: process(CLK, RST)
	begin
		if RST = '1' then
			RX_EMPTY <= '1';
		elsif rising_edge(CLK) then
			if rx_finished = '1' then
				RX_DATA <= rx_buf;
				RX_EMPTY <= '0';
			elsif RX_READ = '1' then
				RX_EMPTY <= '1';
			end if;
		end if;
	end process;

end architecture rtl;

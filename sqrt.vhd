-- megafunction wizard: %ALTSQRT%
-- GENERATION: STANDARD
-- VERSION: WM1.0
-- MODULE: ALTSQRT 

-- ============================================================
-- File Name: sqrt.vhd
-- Megafunction Name(s):
-- 			ALTSQRT
--
-- Simulation Library Files(s):
-- 			altera_mf
-- ============================================================
-- ************************************************************
-- THIS IS A WIZARD-GENERATED FILE. DO NOT EDIT THIS FILE!
--
-- 17.0.2 Build 602 07/19/2017 SJ Lite Edition
-- ************************************************************


--Copyright (C) 2017  Intel Corporation. All rights reserved.
--Your use of Intel Corporation's design tools, logic functions 
--and other software and tools, and its AMPP partner logic 
--functions, and any output files from any of the foregoing 
--(including device programming or simulation files), and any 
--associated documentation or information are expressly subject 
--to the terms and conditions of the Intel Program License 
--Subscription Agreement, the Intel Quartus Prime License Agreement,
--the Intel MegaCore Function License Agreement, or other 
--applicable license agreement, including, without limitation, 
--that your use is for the sole purpose of programming logic 
--devices manufactured by Intel and sold by Intel or its 
--authorized distributors.  Please refer to the applicable 
--agreement for further details.


LIBRARY ieee;
USE ieee.std_logic_1164.all;

LIBRARY altera_mf;
USE altera_mf.all;

ENTITY sqrt IS
	PORT
	(
		clk		: IN STD_LOGIC ;
		radical		: IN STD_LOGIC_VECTOR (23 DOWNTO 0);
		q		: OUT STD_LOGIC_VECTOR (11 DOWNTO 0);
		remainder		: OUT STD_LOGIC_VECTOR (12 DOWNTO 0)
	);
END sqrt;


ARCHITECTURE SYN OF sqrt IS

	SIGNAL sub_wire0	: STD_LOGIC_VECTOR (11 DOWNTO 0);
	SIGNAL sub_wire1	: STD_LOGIC_VECTOR (12 DOWNTO 0);



	COMPONENT altsqrt
	GENERIC (
		pipeline		: NATURAL;
		q_port_width		: NATURAL;
		r_port_width		: NATURAL;
		width		: NATURAL;
		lpm_type		: STRING
	);
	PORT (
			clk	: IN STD_LOGIC ;
			radical	: IN STD_LOGIC_VECTOR (23 DOWNTO 0);
			q	: OUT STD_LOGIC_VECTOR (11 DOWNTO 0);
			remainder	: OUT STD_LOGIC_VECTOR (12 DOWNTO 0)
	);
	END COMPONENT;

BEGIN
	q    <= sub_wire0(11 DOWNTO 0);
	remainder    <= sub_wire1(12 DOWNTO 0);

	ALTSQRT_component : ALTSQRT
	GENERIC MAP (
		pipeline => 1,
		q_port_width => 12,
		r_port_width => 13,
		width => 24,
		lpm_type => "ALTSQRT"
	)
	PORT MAP (
		clk => clk,
		radical => radical,
		q => sub_wire0,
		remainder => sub_wire1
	);



END SYN;

-- ============================================================
-- CNX file retrieval info
-- ============================================================
-- Retrieval info: PRIVATE: INTENDED_DEVICE_FAMILY STRING "MAX 10"
-- Retrieval info: PRIVATE: SYNTH_WRAPPER_GEN_POSTFIX STRING "0"
-- Retrieval info: LIBRARY: altera_mf altera_mf.altera_mf_components.all
-- Retrieval info: CONSTANT: PIPELINE NUMERIC "1"
-- Retrieval info: CONSTANT: Q_PORT_WIDTH NUMERIC "12"
-- Retrieval info: CONSTANT: R_PORT_WIDTH NUMERIC "13"
-- Retrieval info: CONSTANT: WIDTH NUMERIC "24"
-- Retrieval info: USED_PORT: clk 0 0 0 0 INPUT NODEFVAL "clk"
-- Retrieval info: USED_PORT: q 0 0 12 0 OUTPUT NODEFVAL "q[11..0]"
-- Retrieval info: USED_PORT: radical 0 0 24 0 INPUT NODEFVAL "radical[23..0]"
-- Retrieval info: USED_PORT: remainder 0 0 13 0 OUTPUT NODEFVAL "remainder[12..0]"
-- Retrieval info: CONNECT: @clk 0 0 0 0 clk 0 0 0 0
-- Retrieval info: CONNECT: @radical 0 0 24 0 radical 0 0 24 0
-- Retrieval info: CONNECT: q 0 0 12 0 @q 0 0 12 0
-- Retrieval info: CONNECT: remainder 0 0 13 0 @remainder 0 0 13 0
-- Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.vhd TRUE
-- Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.inc FALSE
-- Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.cmp TRUE
-- Retrieval info: GEN_FILE: TYPE_NORMAL sqrt.bsf FALSE
-- Retrieval info: GEN_FILE: TYPE_NORMAL sqrt_inst.vhd FALSE
-- Retrieval info: LIB_FILE: altera_mf
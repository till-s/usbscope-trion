-- Copyright Till Straumann, 2023. Licensed under the EUPL-1.2 or later.
-- You may obtain a copy of the license at
--   https://joinup.ec.europa.eu/collection/eupl/eupl-text-eupl-12
-- This notice must not be removed.

-- THIS FILE WAS AUTOMATICALLY GENERATED; DO NOT EDIT!

-- Generated with: genAppCfgPkgBody.py -p 0x0001 -f AppCfgPkgBody.vhd -S -Lbreak -Lstate -d Till's USB Scope HW V2

library ieee;
use     ieee.std_logic_1164.all;
use     ieee.numeric_std.all;
use     ieee.math_real.all;

use     work.Usb2Pkg.all;
use     work.UlpiPkg.all;
use     work.Usb2UtilPkg.all;
use     work.Usb2DescPkg.all;

package body Usb2AppCfgPkg is
   function usb2AppGetDescriptors return Usb2ByteArray is
      constant c : Usb2ByteArray := (
      -- Usb2DeviceDesc
        0 => x"12",  -- bLength
        1 => x"01",  -- bDescriptorType
        2 => x"00",  -- bcdUSB
        3 => x"02",
        4 => x"ef",  -- bDeviceClass
        5 => x"02",  -- bDeviceSubClass
        6 => x"01",  -- bDeviceProtocol
        7 => x"40",  -- bMaxPacketSize0
        8 => x"09",  -- idVendor
        9 => x"12",
       10 => x"01",  -- idProduct
       11 => x"00",
       12 => x"00",  -- bcdDevice
       13 => x"01",
       14 => x"00",  -- iManufacturer
       15 => x"01",  -- iProduct
       16 => x"00",  -- iSerialNumber
       17 => x"01",  -- bNumConfigurations
      -- Usb2Device_QualifierDesc
       18 => x"0a",  -- bLength
       19 => x"06",  -- bDescriptorType
       20 => x"00",  -- bcdUSB
       21 => x"02",
       22 => x"ef",  -- bDeviceClass
       23 => x"02",  -- bDeviceSubClass
       24 => x"01",  -- bDeviceProtocol
       25 => x"40",  -- bMaxPacketSize0
       26 => x"01",  -- bNumConfigurations
       27 => x"00",  -- bReserved
      -- Usb2ConfigurationDesc
       28 => x"09",  -- bLength
       29 => x"02",  -- bDescriptorType
       30 => x"4b",  -- wTotalLength
       31 => x"00",
       32 => x"02",  -- bNumInterfaces
       33 => x"01",  -- bConfigurationValue
       34 => x"00",  -- iConfiguration
       35 => x"a0",  -- bmAttributes
       36 => x"32",  -- bMaxPower
      -- Usb2InterfaceAssociationDesc
       37 => x"08",  -- bLength
       38 => x"0b",  -- bDescriptorType
       39 => x"00",  -- bFirstInterface
       40 => x"02",  -- bInterfaceCount
       41 => x"02",  -- bFunctionClass
       42 => x"02",  -- bFunctionSubClass
       43 => x"00",  -- bFunctionProtocol
       44 => x"02",  -- iFunction
      -- Usb2InterfaceDesc
       45 => x"09",  -- bLength
       46 => x"04",  -- bDescriptorType
       47 => x"00",  -- bInterfaceNumber
       48 => x"00",  -- bAlternateSetting
       49 => x"01",  -- bNumEndpoints
       50 => x"02",  -- bInterfaceClass
       51 => x"02",  -- bInterfaceSubClass
       52 => x"00",  -- bInterfaceProtocol
       53 => x"00",  -- iInterface
      -- Usb2CDCFuncHeaderDesc
       54 => x"05",  -- bLength
       55 => x"24",  -- bDescriptorType
       56 => x"00",  -- bDescriptorSubtype
       57 => x"20",  -- bcdCDC
       58 => x"01",
      -- Usb2CDCFuncCallManagementDesc
       59 => x"05",  -- bLength
       60 => x"24",  -- bDescriptorType
       61 => x"01",  -- bDescriptorSubtype
       62 => x"00",  -- bmCapabilities
       63 => x"01",  -- bDataInterface
      -- Usb2CDCFuncACMDesc
       64 => x"04",  -- bLength
       65 => x"24",  -- bDescriptorType
       66 => x"02",  -- bDescriptorSubtype
       67 => x"00",  -- bmCapabilities
      -- Usb2CDCFuncUnionDesc
       68 => x"05",  -- bLength
       69 => x"24",  -- bDescriptorType
       70 => x"06",  -- bDescriptorSubtype
       71 => x"00",  -- bControlInterface
       72 => x"01",
      -- Usb2EndpointDesc
       73 => x"07",  -- bLength
       74 => x"05",  -- bDescriptorType
       75 => x"82",  -- bEndpointAddress
       76 => x"03",  -- bmAttributes
       77 => x"08",  -- wMaxPacketSize
       78 => x"00",
       79 => x"ff",  -- bInterval
      -- Usb2InterfaceDesc
       80 => x"09",  -- bLength
       81 => x"04",  -- bDescriptorType
       82 => x"01",  -- bInterfaceNumber
       83 => x"00",  -- bAlternateSetting
       84 => x"02",  -- bNumEndpoints
       85 => x"0a",  -- bInterfaceClass
       86 => x"00",  -- bInterfaceSubClass
       87 => x"00",  -- bInterfaceProtocol
       88 => x"00",  -- iInterface
      -- Usb2EndpointDesc
       89 => x"07",  -- bLength
       90 => x"05",  -- bDescriptorType
       91 => x"81",  -- bEndpointAddress
       92 => x"02",  -- bmAttributes
       93 => x"40",  -- wMaxPacketSize
       94 => x"00",
       95 => x"00",  -- bInterval
      -- Usb2EndpointDesc
       96 => x"07",  -- bLength
       97 => x"05",  -- bDescriptorType
       98 => x"01",  -- bEndpointAddress
       99 => x"02",  -- bmAttributes
      100 => x"40",  -- wMaxPacketSize
      101 => x"00",
      102 => x"00",  -- bInterval
      -- Usb2SentinelDesc
      103 => x"02",  -- bLength
      104 => x"ff",  -- bDescriptorType
      -- Usb2DeviceDesc
      105 => x"12",  -- bLength
      106 => x"01",  -- bDescriptorType
      107 => x"00",  -- bcdUSB
      108 => x"02",
      109 => x"ef",  -- bDeviceClass
      110 => x"02",  -- bDeviceSubClass
      111 => x"01",  -- bDeviceProtocol
      112 => x"40",  -- bMaxPacketSize0
      113 => x"09",  -- idVendor
      114 => x"12",
      115 => x"01",  -- idProduct
      116 => x"00",
      117 => x"00",  -- bcdDevice
      118 => x"01",
      119 => x"00",  -- iManufacturer
      120 => x"01",  -- iProduct
      121 => x"00",  -- iSerialNumber
      122 => x"01",  -- bNumConfigurations
      -- Usb2Device_QualifierDesc
      123 => x"0a",  -- bLength
      124 => x"06",  -- bDescriptorType
      125 => x"00",  -- bcdUSB
      126 => x"02",
      127 => x"ef",  -- bDeviceClass
      128 => x"02",  -- bDeviceSubClass
      129 => x"01",  -- bDeviceProtocol
      130 => x"40",  -- bMaxPacketSize0
      131 => x"01",  -- bNumConfigurations
      132 => x"00",  -- bReserved
      -- Usb2ConfigurationDesc
      133 => x"09",  -- bLength
      134 => x"02",  -- bDescriptorType
      135 => x"4b",  -- wTotalLength
      136 => x"00",
      137 => x"02",  -- bNumInterfaces
      138 => x"01",  -- bConfigurationValue
      139 => x"00",  -- iConfiguration
      140 => x"a0",  -- bmAttributes
      141 => x"32",  -- bMaxPower
      -- Usb2InterfaceAssociationDesc
      142 => x"08",  -- bLength
      143 => x"0b",  -- bDescriptorType
      144 => x"00",  -- bFirstInterface
      145 => x"02",  -- bInterfaceCount
      146 => x"02",  -- bFunctionClass
      147 => x"02",  -- bFunctionSubClass
      148 => x"00",  -- bFunctionProtocol
      149 => x"02",  -- iFunction
      -- Usb2InterfaceDesc
      150 => x"09",  -- bLength
      151 => x"04",  -- bDescriptorType
      152 => x"00",  -- bInterfaceNumber
      153 => x"00",  -- bAlternateSetting
      154 => x"01",  -- bNumEndpoints
      155 => x"02",  -- bInterfaceClass
      156 => x"02",  -- bInterfaceSubClass
      157 => x"00",  -- bInterfaceProtocol
      158 => x"00",  -- iInterface
      -- Usb2CDCFuncHeaderDesc
      159 => x"05",  -- bLength
      160 => x"24",  -- bDescriptorType
      161 => x"00",  -- bDescriptorSubtype
      162 => x"20",  -- bcdCDC
      163 => x"01",
      -- Usb2CDCFuncCallManagementDesc
      164 => x"05",  -- bLength
      165 => x"24",  -- bDescriptorType
      166 => x"01",  -- bDescriptorSubtype
      167 => x"00",  -- bmCapabilities
      168 => x"01",  -- bDataInterface
      -- Usb2CDCFuncACMDesc
      169 => x"04",  -- bLength
      170 => x"24",  -- bDescriptorType
      171 => x"02",  -- bDescriptorSubtype
      172 => x"00",  -- bmCapabilities
      -- Usb2CDCFuncUnionDesc
      173 => x"05",  -- bLength
      174 => x"24",  -- bDescriptorType
      175 => x"06",  -- bDescriptorSubtype
      176 => x"00",  -- bControlInterface
      177 => x"01",
      -- Usb2EndpointDesc
      178 => x"07",  -- bLength
      179 => x"05",  -- bDescriptorType
      180 => x"82",  -- bEndpointAddress
      181 => x"03",  -- bmAttributes
      182 => x"08",  -- wMaxPacketSize
      183 => x"00",
      184 => x"10",  -- bInterval
      -- Usb2InterfaceDesc
      185 => x"09",  -- bLength
      186 => x"04",  -- bDescriptorType
      187 => x"01",  -- bInterfaceNumber
      188 => x"00",  -- bAlternateSetting
      189 => x"02",  -- bNumEndpoints
      190 => x"0a",  -- bInterfaceClass
      191 => x"00",  -- bInterfaceSubClass
      192 => x"00",  -- bInterfaceProtocol
      193 => x"00",  -- iInterface
      -- Usb2EndpointDesc
      194 => x"07",  -- bLength
      195 => x"05",  -- bDescriptorType
      196 => x"81",  -- bEndpointAddress
      197 => x"02",  -- bmAttributes
      198 => x"00",  -- wMaxPacketSize
      199 => x"02",
      200 => x"00",  -- bInterval
      -- Usb2EndpointDesc
      201 => x"07",  -- bLength
      202 => x"05",  -- bDescriptorType
      203 => x"01",  -- bEndpointAddress
      204 => x"02",  -- bmAttributes
      205 => x"00",  -- wMaxPacketSize
      206 => x"02",
      207 => x"00",  -- bInterval
      -- Usb2StringDesc
      208 => x"04",  -- bLength
      209 => x"03",  -- bDescriptorType
      210 => x"09",  -- langID 0x0409
      211 => x"04",
      -- Usb2StringDesc
      212 => x"2e",  -- bLength
      213 => x"03",  -- bDescriptorType
      214 => x"54",  -- T
      215 => x"00",
      216 => x"69",  -- i
      217 => x"00",
      218 => x"6c",  -- l
      219 => x"00",
      220 => x"6c",  -- l
      221 => x"00",
      222 => x"27",  -- '
      223 => x"00",
      224 => x"73",  -- s
      225 => x"00",
      226 => x"20",  --  
      227 => x"00",
      228 => x"55",  -- U
      229 => x"00",
      230 => x"53",  -- S
      231 => x"00",
      232 => x"42",  -- B
      233 => x"00",
      234 => x"20",  --  
      235 => x"00",
      236 => x"53",  -- S
      237 => x"00",
      238 => x"63",  -- c
      239 => x"00",
      240 => x"6f",  -- o
      241 => x"00",
      242 => x"70",  -- p
      243 => x"00",
      244 => x"65",  -- e
      245 => x"00",
      246 => x"20",  --  
      247 => x"00",
      248 => x"48",  -- H
      249 => x"00",
      250 => x"57",  -- W
      251 => x"00",
      252 => x"20",  --  
      253 => x"00",
      254 => x"56",  -- V
      255 => x"00",
      256 => x"32",  -- 2
      257 => x"00",
      -- Usb2StringDesc
      258 => x"1a",  -- bLength
      259 => x"03",  -- bDescriptorType
      260 => x"4d",  -- M
      261 => x"00",
      262 => x"65",  -- e
      263 => x"00",
      264 => x"63",  -- c
      265 => x"00",
      266 => x"61",  -- a
      267 => x"00",
      268 => x"74",  -- t
      269 => x"00",
      270 => x"69",  -- i
      271 => x"00",
      272 => x"63",  -- c
      273 => x"00",
      274 => x"61",  -- a
      275 => x"00",
      276 => x"20",  --  
      277 => x"00",
      278 => x"41",  -- A
      279 => x"00",
      280 => x"43",  -- C
      281 => x"00",
      282 => x"4d",  -- M
      283 => x"00",
      -- Usb2SentinelDesc
      284 => x"02",  -- bLength
      285 => x"ff"   -- bDescriptorType
      );
   begin
      return c;
   end function usb2AppGetDescriptors;
end package body Usb2AppCfgPkg;

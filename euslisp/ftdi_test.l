#!/usr/bin/env roseus

# you need to install JSK private rcb4eus/ftdi
(require "~/prog/rcb4eus/ftdi")

(setq fcon (instantiate ftdi_context))
(setq interface 0
      vid #x0403
      pid #x6015
      description 0
      serial 0
      index 0
      baud 921600
      bits 8
      stop 0
      parity 0
      )
(format t "; ftdi_init=~A~%" (ftdi_init fcon))
(format t "; ftdi_set_interface=~A~%" (ftdi_set_interface fcon interface))
(format t "; ftdi_usb_open_desk_index=~A~%"
    (ftdi_usb_open_desc_index fcon vid pid description serial index)
(format t "; ftdi_set_baudrate=~A~%" (ftdi_set_baudrate fcon baud))
(format t "; ftdi_set_line_property=~A~%"
    (ftdi_set_line_property fcon bits stop parity))
(ftdi_set_latency_timer fcon 1)
(setq msg (list #xe8 0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62 63 64 65))
(setq i (ftdi_write_data fcon (coerce msg string) (length msg)))
(unless (= (length msg) i)
  (format t ";; (ftdi_write_data ~A ~A)~%" (length msg) i)
  )
(setq buf (instantiate string 80))
(while (= 0 (setq i (ftdi_read_data fcon buf (length buf)))))
(format t "~A~%" (coerce buf cons))

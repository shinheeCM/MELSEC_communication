# MELSEC Communication Guide

## PLC Register Overview

This guide provides an overview of the PLC registers used for communication between a PC and a PLC in a conveyor system setup.

## pymcprotocol
 - MC protocol(MELSEC Communication Protocol) implementation by Python.
 - MC protocol enables you to operate PLC from computer.

## Installation
 - pip install pymcprotocol
 
### Register Description

#### PC to PLC Communication
- **D2000**: Communication signal from PC to PLC.
- **D2001**: Signal indicating arrival of a conveyor.
- **D2002**: Signal indicating culinary (or potentially culinary) operations on conveyor B.
- **D2003**: Signal indicating movement of conveyor A.
- **D2004**: Signal indicating movement of conveyor B.
- **D2005**: Signal indicating presence or absence of product on the conveyor.
- **D2006**: Signal indicating completion of product movement on conveyor A (input).
- **D2007**: Signal indicating completion of product movement on conveyor B (discharge).
- **D2008 to D2009**: Reserved for future use or spare.

#### PLC to PC Communication
- **D2010**: Communication signal from PLC to PC.
- **D2011**: Signal indicating the automatic operation of the conveyor.
- **D2012**: Ready signal for conveyor A (input).
- **D2013**: Ready signal for conveyor B (discharge).
- **D2014**: Signal indicating completion of movement on conveyor A (input).
- **D2015**: Signal indicating completion of movement on conveyor B (discharge).

### Register Use Cases

These registers facilitate the communication and control signals between the PC and PLC, ensuring smooth operations of the conveyor system. Each register has a specific role, contributing to the automation and monitoring of conveyor processes.

### Example Usage

When implementing communication using these registers, the PC sends control signals to the PLC by writing to the designated registers (D2000 to D2009). The PLC, in turn, responds with status updates and other signals by writing to the registers (D2010 to D2015), which the PC can read.

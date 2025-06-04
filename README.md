# MELSEC Communication Guide

# API LIST
```
    ./status.sh

    Get status:
    curl http://localhost:5000/status

    Start Input Conveyor:
    curl -X POST http://localhost:5000/start_input_conveyor

    

    Start toggle signal (D2000, D2010):
    curl -X POST http://localhost:5000/toggle_signal


    To call your POST API at /amr/product-detected, you can use curl like this:

    curl -X POST http://localhost:5000/amr/product-detected \
        -H "Content-Type: application/json" \
        -d '{"product_detected": 1}'

    This will set D2005 = 1 (product detected).

    To clear it (set D2005 = 0):

    curl -X POST http://localhost:5000/amr/product-detected \
        -H "Content-Type: application/json" \
        -d '{"product_detected": 0}'

    The PLC to detect no object on conveyor (based on the signals), and when this happens, automatically call (trigger) the AMR via the MES or a button API.

    curl http://localhost:5000/amr/object-detection-status

        {"no_object_detected":true,"object_detected":false}

    AMR arrived -->
        curl -X POST http://localhost:5000/amr/arrived \
            -H "Content-Type: application/json" \
            -d '{"type": "loading"}'
        
        curl -X POST http://localhost:5000/amr/arrived \
            -H "Content-Type: application/json" \
            -d '{"type": "unloading"}'
    

    Align-conveyor -->
        curl -X POST http://localhost:5000/amr/align-conveyor \
            -H "Content-Type: application/json" \
            -d '{"type": "loading"}'
        
        curl -X POST http://localhost:5000/amr/align-conveyor \
            -H "Content-Type: application/json" \
            -d '{"type": "unloading"}'
    

    Align-conveyor Confirmed-->
        curl "http://localhost:5000/amr/alignment/confirmed?type=loading"
        curl "http://localhost:5000/amr/alignment/confirmed?type=unloading"



    NOW run convare from AMR (Naresh Action server)

    curl -X POST http://localhost:5000/amr/confirm-product

    Move to Point 1 → Stop for 3s → Move to Point 2 → Stop for 3s → Move to Point 3 → Stop for 3s → Return to Home → Stop



    curl http://localhost:5000/amr/object-detection-status

        {"no_object_detected":false,"object_detected":true}
    Issue movement command for Discharge Conveyor (B)
    Send the Amr to Convare B
    then call -->

    Start Discharge Conveyor:
    curl -X POST http://localhost:5000/start_discharge_conveyor


    curl -X POST http://localhost:5000/plc/confirm-product
{"product_confirmed":true}


    

```
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
- **D2011**: Signal indicating the automatic operation of the conveyor. (alarm)
- **D2012**: Ready signal for conveyor A (input).
- **D2013**: Ready signal for conveyor B (discharge).
- **D2014**: Signal indicating completion of movement on conveyor A (input).
- **D2015**: Signal indicating completion of movement on conveyor B (discharge).

### Register Use Cases

These registers facilitate the communication and control signals between the PC and PLC, ensuring smooth operations of the conveyor system. Each register has a specific role, contributing to the automation and monitoring of conveyor processes.

### Example Usage

When implementing communication using these registers, the PC sends control signals to the PLC by writing to the designated registers (D2000 to D2009). The PLC, in turn, responds with status updates and other signals by writing to the registers (D2010 to D2015), which the PC can read.

# CDC-Safe-Ping-Pong-Buffer
Full RTL implementation of a CDC-safe ping-pong buffer with dual-bank BRAM and toggle-based handshaking. Includes SymbiYosys formal verification: bounded safety properties (no overwrite/tearing) plus unbounded liveness (eventual frame handoff under fairness).



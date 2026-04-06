= nrai_ADS-DV_CAN-API
This is the Newcastle Racing AI (nrai) implementation of the ADS-DV software
interface specification (including all other sensors accessed via CAN e.g. GPS).
it aims to be as minimal as possible, facilitating on the bare minimum to pack
and unpack CAN frames, only implementing checks where required for the packing
of frames (e.g. stat clamping). All data sanity checks are expected to be 
implemented by the calling program. 

== Usage
This API provides 2 structs for storing data to be written to, and received from
the VCU over CAN. They are `nrai_can_ai_read` and `nrai_can_ai_write` for data
received from the VCU and data to write to the VCU respectively.
This API provides functions for the AI to pack data into CAN frames, named in
the format 
`int32_t nrai_can_mkframe_<frame_name>(struct nrai_can_ai_write *, struct can_frame *);`
and function for the AI to unpack CAN frames, named in the format
`int32_t nrai_can_unpack_<frame_name>(struct nrai_can_ai_read *, struct can_frame *);`.
This API also provides an enum for translating CAN message names into their ID,
all entry's are named in the format `NRAI_CAN_ID_<frame_name>`

== Notes
This API makes use of limited use of C23 extensions for the purpose of either
improving clarity or simplifying code. While all C23 extensions used are 
supported by GCC and Clang, not all compilers support them.

import struct 

data = [
    999,999,999,999,999, #5i
    0,0,0,0,0, #5?

    "RIGHT_CORNER".encode(), #C State 21
    "STRAIGHT".encode(), #N State 
    "E".encode(), #C Heading
    "S".encode(), #N Headign
    "E-S".encode(), #C Pos 
    "S-Z".encode(), #N Pos
]
# input_string[:21].ljust(21, b'\0'.decode())

format_string = "<5i5?12s12s1s1s3s3s"
# Pack the data into bytes
packed_data = struct.pack(format_string, *data)

print("Packed bytes:", packed_data)

print(len(data), len(packed_data))

print(struct.unpack(format_string, packed_data))
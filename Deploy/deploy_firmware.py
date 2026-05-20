import os
import struct
from coincurve import PrivateKey
from ecdh_aesgcm import encrypt_with_eph_and_nonce

ZERO = b'\x00'
GROUP_ORDER = (
    b'\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff\xff'
    b'\xfe\xba\xae\xdc\xe6\xafH\xa0;\xbf\xd2^\x8c\xd06AA'
)

def get_valid_secret() -> bytes:
    """
    Generate a random 32-byte scalar in range (0, GROUP_ORDER).
    """
    while True:
        secret = os.urandom(32)
        if ZERO < secret < GROUP_ORDER:
            return secret

def encrypt_firmware_with_key(
    fw_file: str,
    fw_name: str,
    pk_file: str,
    pk_name: str,
    output_dir: str = "Firmware_To_Deploy"
):
    """
    Reads a firmware file and a public key file from disk,
    encrypts the firmware, creates a signed header, and writes an .sfb
    to output_dir/{pk_name}_{fw_name}.sfb.
    """
    print("-------------------------------------------------")
    print(f"Encrypting {fw_name} with public key {pk_name}")
    print("-------------------------------------------------")

    # 1) Read receiver's public key
    with open(pk_file, 'rb') as f:
        receiver_pub_bytes = f.read()

    # 2) Generate ephemeral key + nonce
    eph_sk_bytes = get_valid_secret()
    eph_sk = PrivateKey(eph_sk_bytes)
    nonce = os.urandom(12)

    # 3) Read the firmware data
    with open(fw_file, 'rb') as f:
        firmware_data = f.read()

    # 4) Encrypt firmware
    enc_fw = encrypt_with_eph_and_nonce(
        receiver_pubkey_bytes=receiver_pub_bytes,
        plaintext=firmware_data,
        ephemeral_sk=eph_sk,
        nonce=nonce
    )
    # parse out ephemeral pub, nonce, tag, ciphertext
    fw_eph_pub   = enc_fw[1:65]   # ignoring leading 0x04
    fw_nonce     = enc_fw[65:77]
    fw_tag       = enc_fw[77:93]
    fw_encrypted = enc_fw[93:]

    print("Firmware ephemeral public key:", fw_eph_pub.hex())
    print("Firmware nonce:               ", fw_nonce.hex())
    print("Firmware tag:                 ", fw_tag.hex())

    # 5) Create the SFU header.
    header_binary = b"SFU1" + struct.pack(
        "HHIII", 1, 1, len(firmware_data), 0, len(firmware_data)
    )
    header_binary += fw_tag
    header_binary += fw_tag
    header_binary += fw_nonce
    header_binary += fw_eph_pub
    header_binary += b"\x00" * 48

    # 6) Generate a GMAC tag over this header by encrypting an empty payload
    #    with the same ephemeral key + nonce, passing the header as AAD
    enc_hdr = encrypt_with_eph_and_nonce(
        receiver_pubkey_bytes=receiver_pub_bytes,
        plaintext=b"",
        ephemeral_sk=eph_sk,
        nonce=nonce,
        aad=header_binary
    )
    hdr_tag = enc_hdr[77:93]  # The 16-byte AES-GCM tag

    print("Header GMAC tag:", hdr_tag.hex())

    # Append the GMAC tag to the header
    header_binary += hdr_tag

    # Fill out the rest of the header
    header_binary += b'\xff' * 32
    header_binary += b'\xff' * 64
    header_binary += b'\x00' * 32
    header_binary += b'\xff' * 192

    # 7) Write .sfb file
    os.makedirs(output_dir, exist_ok=True)
    output_file = f"{output_dir}/{pk_name}_{fw_name}.sfb"
    with open(output_file, 'wb') as out:
        out.write(header_binary)
        out.write(fw_encrypted)

    print(f"Wrote {output_file}\n")

def main():
    fw_files = [
        ('Firmware_As_Built/UserApp.bin', 'UserApp'),
    ]

    pk_files = [
        ('Public_Keys/pub_key_b2.bin', 'B2'),
        ('Public_Keys/pub_key_b3.bin', 'B3'),
        ('Public_Keys/pub_key_b4.bin', 'B4'),
        ('Public_Keys/pub_key_b5.bin', 'B5'),
        ('Public_Keys/pub_key_b6.bin', 'B6')
    ]

    for fw_file, fw_name in fw_files:
        for pk_file, pk_name in pk_files:
            encrypt_firmware_with_key(fw_file, fw_name, pk_file, pk_name)

if __name__ == "__main__":
    main()

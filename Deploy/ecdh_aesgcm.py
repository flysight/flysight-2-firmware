import os
from coincurve import PrivateKey, PublicKey
from Crypto.Cipher import AES
from Crypto.Protocol.KDF import HKDF
from Crypto.Hash import SHA512

def ecdh_derive_key_sha512(
    sender_sk: PrivateKey,
    receiver_pubkey_bytes: bytes,
    key_len: int = 32
) -> bytes:
    """
    Perform ECDH with `sender_sk` (our ephemeral private key)
    and `receiver_pubkey_bytes`. Then derive a `key_len`-byte
    AES key with HKDF-SHA512.
    """
    # 1. Convert the receiver’s public key bytes into a coincurve.PublicKey
    receiver_pub = PublicKey(receiver_pubkey_bytes)

    # 2. ECDH: multiply ephemeral_secret * receiver_pub
    shared_point = receiver_pub.multiply(sender_sk.secret)

    # 3. Get our local ephemeral pubkey (uncompressed, 65 bytes)
    local_pubkey_bytes = sender_sk.public_key.format(compressed=False)

    # 4. Concatenate local ephemeral pubkey + the shared point (also uncompressed)
    #    to form the HKDF "master" input
    master = local_pubkey_bytes + shared_point.format(compressed=False)

    # 5. Derive final AES key with HKDF-SHA512
    derived_key = HKDF(
        master=master,
        key_len=key_len,
        salt=b"",
        hashmod=SHA512
    )

    return derived_key

def encrypt_with_eph_and_nonce(
    receiver_pubkey_bytes: bytes,
    plaintext: bytes,
    ephemeral_sk: PrivateKey,
    nonce: bytes,
    aad: bytes = b""
) -> bytes:
    """
    Custom ECIES-like encryption:
      - ephemeral_sk: Your ephemeral PrivateKey
      - receiver_pubkey_bytes: The target's public key (bytes)
      - nonce: 12 bytes for AES-GCM
      - aad: optional additional authenticated data (header, etc.)

    Returns: ephemeral_pub (65 bytes, uncompressed) + nonce(12) + tag(16) + ciphertext
    """
    # 1. Derive AES-GCM key via ECDH + HKDF(SHA512)
    aes_key = ecdh_derive_key_sha512(ephemeral_sk, receiver_pubkey_bytes, key_len=32)

    # 2. Create AES-GCM cipher
    cipher = AES.new(aes_key, AES.MODE_GCM, nonce=nonce)
    if aad:
        cipher.update(aad)  # AAD is authenticated but not encrypted

    # 3. Encrypt the plaintext
    ciphertext = cipher.encrypt(plaintext)
    tag = cipher.digest()

    # 4. Return ephemeral pubkey + nonce + tag + ciphertext
    ephemeral_pub_bytes = ephemeral_sk.public_key.format(compressed=False)  # 65 bytes, uncompressed
    return ephemeral_pub_bytes + nonce + tag + ciphertext

def decrypt_with_eph_and_nonce(
    receiver_sk: PrivateKey,
    data: bytes,
    aad: bytes = b""
) -> bytes:
    """
    Inverse of encrypt_with_eph_and_nonce, using the new HKDF input:
      master = ephemeral_pub(65) + shared_point(65).
    data layout: ephemeral_pub(65) + nonce(12) + tag(16) + ciphertext
    """
    if len(data) < 65 + 12 + 16:
        raise ValueError("Invalid data length")

    # 1) Parse the input
    ephemeral_pub_bytes = data[0:65]
    nonce = data[65:77]
    tag   = data[77:93]
    ciphertext = data[93:]

    # 2) ECDH: ephemeral_pub * receiver_sk
    ephemeral_pub = PublicKey(ephemeral_pub_bytes)
    shared_point = ephemeral_pub.multiply(receiver_sk.secret)

    # 3) Build the HKDF input:
    #    ephemeral_pub_bytes + uncompressed shared_point
    master = ephemeral_pub_bytes + shared_point.format(compressed=False)

    # 4) Derive the AES key with HKDF-SHA512
    aes_key = HKDF(
        master=master,
        key_len=32,
        salt=b"",
        hashmod=SHA512
    )

    # 5) AES-GCM decrypt
    cipher = AES.new(aes_key, AES.MODE_GCM, nonce=nonce)
    if aad:
        cipher.update(aad)  # Additional Authenticated Data
    plaintext = cipher.decrypt(ciphertext)
    cipher.verify(tag)  # raises ValueError if tag mismatch
    return plaintext

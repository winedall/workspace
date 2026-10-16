#!/bin/bash
# scripts/signing/keygen.sh
# Generates Ed25519 key pair for release package signing.
# Run once during OTA system setup.
set -euo pipefail

KEY_DIR="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$KEY_DIR"

export KEY_DIR
python3 -c "
import os
from nacl.bindings import crypto_sign_keypair

key_dir = os.environ['KEY_DIR']
pk, sk = crypto_sign_keypair()
with open(os.path.join(key_dir, 'public_key.key'), 'wb') as f:
    f.write(pk)
with open(os.path.join(key_dir, 'private_key.key'), 'wb') as f:
    f.write(sk)
os.chmod(os.path.join(key_dir, 'private_key.key'), 0o600)
"
echo "Keys generated in $KEY_DIR"

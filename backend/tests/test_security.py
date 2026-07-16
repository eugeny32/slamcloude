from app.security import API_KEY_PREFIX, generate_api_key, hash_api_key


def test_generated_keys_are_prefixed_and_unique() -> None:
    keys = {generate_api_key() for _ in range(100)}
    assert len(keys) == 100
    assert all(k.startswith(API_KEY_PREFIX) for k in keys)


def test_hash_is_deterministic_hex64() -> None:
    key = generate_api_key()
    h1, h2 = hash_api_key(key), hash_api_key(key)
    assert h1 == h2
    assert len(h1) == 64
    assert int(h1, 16)  # valid hex


def test_different_keys_different_hashes() -> None:
    assert hash_api_key("sk_a") != hash_api_key("sk_b")

"""Admin CLI (dev bootstrap): uv run python -m app.cli create-user --email x@y.z"""

import argparse
import asyncio
import uuid

from sqlalchemy.exc import IntegrityError

from app.db import SessionLocal
from app.models import PlanTier, User
from app.security import generate_api_key, hash_api_key


async def _create_user(email: str, plan: str) -> None:
    api_key = generate_api_key()
    user = User(
        id=uuid.uuid4(),
        email=email,
        api_key_hash=hash_api_key(api_key),
        plan_tier=PlanTier(plan),
    )
    async with SessionLocal() as session:
        session.add(user)
        try:
            await session.commit()
        except IntegrityError:
            raise SystemExit(f"User with email {email!r} already exists") from None
    print(f"User created: {email} (plan: {plan})")
    print(f"API key (shown once, store it now): {api_key}")


def main() -> None:
    parser = argparse.ArgumentParser(prog="slamcloude-cli")
    sub = parser.add_subparsers(dest="command", required=True)

    create = sub.add_parser("create-user", help="Create a user and print its API key")
    create.add_argument("--email", required=True)
    create.add_argument(
        "--plan", default="free", choices=[t.value for t in PlanTier]
    )

    args = parser.parse_args()
    if args.command == "create-user":
        asyncio.run(_create_user(args.email, args.plan))


if __name__ == "__main__":
    main()

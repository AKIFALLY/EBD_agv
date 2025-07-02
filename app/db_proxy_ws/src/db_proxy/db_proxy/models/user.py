from typing import Optional
from sqlmodel import SQLModel, Field
from datetime import datetime, timezone
from sqlalchemy import Column, DateTime, String
from pydantic import ConfigDict


class User(SQLModel, table=True):
    __tablename__ = "user"
    id: Optional[int] = Field(default=None, primary_key=True)
    username: str = Field(sa_column=Column(
        String(50), unique=True, nullable=False))
    email: Optional[str] = Field(
        default=None, sa_column=Column(String(100), unique=True))
    password_hash: str = Field(sa_column=Column(String(255), nullable=False))
    full_name: Optional[str] = Field(
        default=None, sa_column=Column(String(100)))
    role: str = Field(default="user", sa_column=Column(
        String(20), nullable=False))  # admin, user, operator
    is_active: bool = Field(default=True)
    created_at: datetime = Field(
        sa_column=Column(DateTime(timezone=True), nullable=False),
        default_factory=lambda: datetime.now(timezone.utc))
    updated_at: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))
    last_login: Optional[datetime] = Field(
        default=None,
        sa_column=Column(DateTime(timezone=True), nullable=True))

    model_config = ConfigDict(from_attributes=True)

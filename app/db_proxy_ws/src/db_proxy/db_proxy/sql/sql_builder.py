from typing import Type, Any
from sqlmodel import SQLModel, select
from sqlalchemy import insert, update, delete
from sqlalchemy.dialects import postgresql


def query_to_str(stmt) -> str:
    """將 SQLAlchemy 查詢語句轉換為 SQL 字串（帶入實際值）"""
    return str(stmt.compile(dialect=postgresql.dialect(), compile_kwargs={"literal_binds": True}))


class SQLBuilder:
    @staticmethod
    def select_by(model: Type[SQLModel], **filters) -> str:
        stmt = select(model)
        for attr, value in filters.items():
            stmt = stmt.where(getattr(model, attr) == value)
        return query_to_str(stmt)

    @staticmethod
    def insert_into(model: Type[SQLModel], **values) -> str:
        stmt = insert(model).values(**values)
        return query_to_str(stmt)

    @staticmethod
    def update_by(model: Type[SQLModel], where: dict, values: dict) -> str:
        stmt = update(model)
        for attr, value in where.items():
            stmt = stmt.where(getattr(model, attr) == value)
        stmt = stmt.values(**values)
        return query_to_str(stmt)

    @staticmethod
    def delete_by(model: Type[SQLModel], **filters) -> str:
        stmt = delete(model)
        for attr, value in filters.items():
            stmt = stmt.where(getattr(model, attr) == value)
        return query_to_str(stmt)
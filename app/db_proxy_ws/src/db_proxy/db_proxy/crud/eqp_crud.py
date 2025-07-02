from db_proxy.models import Eqp,EqpSignal,EqpPort
from db_proxy.crud.base_crud import BaseCRUD
from sqlalchemy import select
from sqlmodel import Session

eqp_crud = BaseCRUD(Eqp, id_column="id")
eqp_port_crud = BaseCRUD(EqpPort, id_column="id")

class EqpSignalCRUD(BaseCRUD):
    def get_signals_with_dm(self, session: Session):
        stmt = select(EqpSignal).where(EqpSignal.dm_address != None)
        # print(f"[DEBUG] SQL statement: {stmt}")
        result = session.exec(stmt).all()
        # for i, r in enumerate(result):
        #     print(f"[DEBUG] Row {i}: type={type(r)}, value={r}")
        return result
eqp_signal_crud = EqpSignalCRUD(EqpSignal, id_column="id")
from db_proxy.models import NodeType
from db_proxy.models import Node
from db_proxy.models import Edge
from db_proxy.models import KukaNode
from db_proxy.models import KukaEdge
from db_proxy.crud.base_crud import BaseCRUD

node_type_crud = BaseCRUD(NodeType, id_column="id")
node_crud = BaseCRUD(Node, id_column="id")
edge_crud = BaseCRUD(Edge, id_column="id")
kuka_node_crud = BaseCRUD(KukaNode, id_column="id")
kuka_edge_crud = BaseCRUD(KukaEdge, id_column="id")

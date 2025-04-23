import os
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sqlite3 import Connection, Cursor, connect, Error


class WormholeManager:
    """Handle database connections and wormhole locations

    Attributes
    ----------
    node: Node
        ROS node for creating all clients
    
    db_path: str
        Path to the database
    
    conn: Connection
        Database connection
    
    cursor: Cursor
        Database connection curser

    Methods
    -------
    _init_db(self) -> None:
        Initialize Database

    _insert_initial_wormholes(self) -> None:
        Insert initial coordinates

    _add_wormhole(self, from_map: str, to_map: str, x: float, y: float) -> None:
        Add wormhole locations to database

    get_wormhole_pose(self, from_map: str, to_map: str) -> PoseStamped | None:
        Fetch wormhole locations from the database
    """

    __slots__: list[str] = [
        'node',
        'db_path',
        'conn',
        'cursor'
    ]

    def __init__(self, node: Node) -> None:
        self.node: Node = node

        self.db_path: str = os.path.join(
            os.path.expanduser("~"), "wormholes.db")
        db_exists: bool = os.path.exists(self.db_path)

        self.conn: Connection = connect(self.db_path)
        self.cursor: Cursor = self.conn.cursor()

        self._init_db()

        if not db_exists:
            self.node.get_logger().info(
                "[WormholeManager] Database not found. Creating and adding initial wormholes.")
            self._insert_initial_wormholes()
        else:
            self.node.get_logger().info(
                "[WormholeManager] Existing database found. Using current wormholes.")

    def _init_db(self) -> None:
        """Initialize Database"""
        self.cursor.execute("""
            CREATE TABLE IF NOT EXISTS wormholes (
                from_map TEXT,
                to_map TEXT,
                x REAL,
                y REAL,
                PRIMARY KEY (from_map, to_map)
            )
        """)
        self.conn.commit()

    def _insert_initial_wormholes(self) -> None:
        """Insert initial coordinates"""
        self._add_wormhole('map1', 'map2', -7.0, -2.0)
        self._add_wormhole('map2', 'map1', 7.0, -4.0)

    def _add_wormhole(self, from_map: str, to_map: str, x: float, y: float) -> None:
        """Add wormhole locations to database

        Parameters
        ----------
        from_map : str
            From map name
        to_map : str
            To map name
        x : float
            X Coordinates
        y : float
            Y Coordinates
        """
        try:
            self.cursor.execute("""
                INSERT OR IGNORE INTO wormholes (from_map, to_map, x, y)
                VALUES (?, ?, ?, ?)
            """, (from_map, to_map, x, y))
            self.conn.commit()
        except Error as e:
            self.node.get_logger().error(
                f"[DB] Failed to insert wormhole: {e}")

    def get_wormhole_pose(self, from_map: str, to_map: str) -> PoseStamped | None:
        """Fetch wormhole locations from the database

        Parameters
        ----------
        from_map : str
            current map
        to_map : str
            Target map

        Returns
        -------
        PoseStamped | None
            Coordinates as PostStamped messages
        """
        try:
            self.cursor.execute("""
                SELECT x, y FROM wormholes WHERE from_map = ? AND to_map = ?
            """, (from_map, to_map))
            if result := self.cursor.fetchone():
                x, y = result
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0

                self.node.get_logger().info(
                    f"[WormholeManager] Found wormhole: {from_map} -> {to_map}")
                return pose

            self.node.get_logger().warn(
                f"[WormholeManager] No wormhole defined between '{from_map}' and '{to_map}'")
            return None

        except Error as e:
            self.node.get_logger().error(f"[DB] Failed to fetch wormhole: {e}")
            return None

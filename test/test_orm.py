import unittest
import pycram.orm.base
import pycram.orm.task
import pycram.orm.object_designator
import pycram.orm.motion_designator
import pycram.orm.action_designator
import pycram.task
from pycram.task import with_tree
import test_bullet_world
import test_task_tree
import anytree
import sqlalchemy
import sqlalchemy.orm
import os


class ORMTestSchema(unittest.TestCase):

    def test_schema_creation(self):
        self.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session.commit()
        tables = list(pycram.orm.base.Base.metadata.tables.keys())
        self.assertTrue("Position" in tables)
        self.assertTrue("Quaternion" in tables)
        self.assertTrue("TaskTreeNode" in tables)
        self.assertTrue("Code" in tables)
        self.assertTrue("Action" in tables)
        self.assertTrue("ParkArms" in tables)
        self.assertTrue("Navigate" in tables)
        self.assertTrue("MoveTorso" in tables)
        self.assertTrue("SetGripper" in tables)
        self.assertTrue("Release" in tables)
        self.assertTrue("Grip" in tables)
        self.assertTrue("PickUp" in tables)
        self.assertTrue("Place" in tables)
        self.assertTrue("Transport" in tables)
        self.assertTrue("LookAt" in tables)
        self.assertTrue("Detect" in tables)
        self.assertTrue("Open" in tables)
        self.assertTrue("Close" in tables)


class ORMTaskTreeTestCase(test_task_tree.TaskTreeTestCase):

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        cls.engine = sqlalchemy.create_engine("sqlite+pysqlite:///:memory:", echo=False)

    def setUp(self):
        super().setUp()
        pycram.orm.base.Base.metadata.create_all(self.engine)
        self.session = sqlalchemy.orm.Session(bind=self.engine)
        self.session.commit()

    def tearDown(self):
        super().tearDown()
        self.session.close()

    def test_node(self):
        """Test if the objects in the database is equal with the objects that got serialized."""
        self.plan()
        pycram.task.task_tree.root.insert(self.session)

        node_results = self.session.query(pycram.orm.task.TaskTreeNode).all()
        self.assertEqual(len(node_results), len(pycram.task.task_tree.root))

        code_results = self.session.query(pycram.orm.task.Code).all()
        self.assertEqual(len(code_results), len(pycram.task.task_tree.root))

        position_results = self.session.query(pycram.orm.base.Position).all()
        self.assertEqual(len(position_results), 1)

        quaternion_results = self.session.query(pycram.orm.base.Quaternion).all()
        self.assertEqual(len(quaternion_results), 1)

        park_arms_results = self.session.query(pycram.orm.action_designator.ParkArmsAction).all()
        self.assertEqual(len(park_arms_results), 2)

        navigate_results = self.session.query(pycram.orm.action_designator.NavigateAction).all()
        self.assertEqual(len(navigate_results), 1)

        action_results = self.session.query(pycram.orm.action_designator.Action).all()
        self.assertEqual(len(action_results), 3)

    @classmethod
    def TearDownClass(cls):
        super().TearDownClass()
        pycram.orm.base.Base.metadata.drop_all(cls.engine)
        cls.session.commit()


if __name__ == '__main__':
    unittest.main()

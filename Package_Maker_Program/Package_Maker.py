#!/usr/bin/env python3
import sys
import os
import subprocess
from pathlib import Path
from PyQt5.QtWidgets import (
    QApplication, QWizard, QWizardPage, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QTextEdit, QCheckBox, QComboBox, QButtonGroup
)
import textwrap

class MetadataPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Package Metadata")
        layout = QVBoxLayout()

        # Category checkboxes
        self.comm_cb    = QCheckBox("Communication")
        self.vision_cb  = QCheckBox("Vision")
        self.control_cb = QCheckBox("Control")
        self.safety_cb  = QCheckBox("Safety")
        cb_layout = QHBoxLayout()
        for cb in (self.comm_cb, self.vision_cb, self.control_cb, self.safety_cb):
            cb_layout.addWidget(cb)
        layout.addLayout(cb_layout)

        # Make them mutually exclusive and track completeness
        self.cat_group = QButtonGroup(self)
        self.cat_group.setExclusive(True)
        for cb in (self.comm_cb, self.vision_cb, self.control_cb, self.safety_cb):
            self.cat_group.addButton(cb)
            cb.toggled.connect(self.completeChanged)

        # Package name
        layout.addWidget(QLabel("Package Name (Example: package_name):"))
        self.pkg_edit = QLineEdit()
        layout.addWidget(self.pkg_edit)
        self.registerField("package_name*", self.pkg_edit)

        # Author
        layout.addWidget(QLabel("Author Name:"))
        self.author_edit = QLineEdit()
        layout.addWidget(self.author_edit)
        self.registerField("author*", self.author_edit)

        layout.addWidget(QLabel("Author Email:"))
        self.email_edit = QLineEdit()
        layout.addWidget(self.email_edit)
        self.registerField("email*", self.email_edit)

        # Descriptions
        layout.addWidget(QLabel("Short Description:"))
        self.desc_edit = QLineEdit()
        layout.addWidget(self.desc_edit)
        self.registerField("description*", self.desc_edit)

        layout.addWidget(QLabel("Detailed Purpose / README Intro:"))
        self.purpose_edit = QTextEdit()
        layout.addWidget(self.purpose_edit)
        self.registerField("purpose", self.purpose_edit, "plainText", self.purpose_edit.textChanged)

        self.setLayout(layout)

    def isComplete(self):
        # require exactly one category AND all registered fields filled
        checked = sum(cb.isChecked() for cb in
                      (self.comm_cb, self.vision_cb, self.control_cb, self.safety_cb))
        return checked == 1 and super().isComplete()

class NodesPage(QWizardPage):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTitle("Node & Launch Configuration")
        self.node_edits = []
        layout = QVBoxLayout()

        layout.addWidget(QLabel("Number of nodes (1–5):"))
        self.count_cb = QComboBox()
        self.count_cb.addItems([str(i) for i in range(1,6)])
        layout.addWidget(self.count_cb)
        self.count_cb.currentIndexChanged.connect(self.update_node_fields)

        self.fields_layout = QVBoxLayout()
        layout.addLayout(self.fields_layout)

        layout.addWidget(QLabel("Launch file name without extension:"))
        self.launch_edit = QLineEdit()
        layout.addWidget(self.launch_edit)
        self.registerField("launch_file*", self.launch_edit)

        self.setLayout(layout)
        self.update_node_fields(0)

    def update_node_fields(self, idx):
        for i in reversed(range(self.fields_layout.count())):
            self.fields_layout.itemAt(i).widget().deleteLater()
        self.node_edits.clear()
        count = int(self.count_cb.currentText())
        for i in range(count):
            lbl = QLabel(f"Node {i+1} filename (e.g. talker.py):")
            edit = QLineEdit()
            self.fields_layout.addWidget(lbl)
            self.fields_layout.addWidget(edit)
            self.node_edits.append(edit)

class NodeMakerWizard(QWizard):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS 2 Node Maker")
        self.addPage(MetadataPage())
        self.addPage(NodesPage())
        self.button(self.FinishButton).clicked.connect(self.generate_package)

    def generate_package(self):
        # Gather fields
        pkg      = self.field("package_name")
        author   = self.field("author")
        email    = self.field("email")
        desc     = self.field("description")
        purpose  = self.field("purpose")
        launch   = self.field("launch_file")
        page2    = self.page(1)
        nodes    = [e.text().strip() for e in page2.node_edits]

        base    = Path.cwd() / pkg
        src_pkg = base / pkg
        for p in [src_pkg, base/"launch", base/"resource", base/"test"]:
            p.mkdir(parents=True, exist_ok=True)

        # Optional folders
        md = self.page(0)
        if md.vision_cb.isChecked():
            (src_pkg/"models").mkdir(exist_ok=True)
        if md.safety_cb.isChecked():
            (src_pkg/"logs").mkdir(exist_ok=True)
        if md.comm_cb.isChecked():
            (src_pkg/"system_msgs").mkdir(exist_ok=True)

        # resource marker
        (base/"resource"/pkg).write_text("")

        # __init__.py
        (src_pkg/"__init__.py").write_text("# package init\n")

        # Node files + entry points
        entry_pts = []
        for fn in nodes:
            mod = fn.replace(".py","")
            entry_pts.append(f"            '{mod} = {pkg}.{mod}:main',")
            node_path = src_pkg/fn
            node_path.write_text(textwrap.dedent(f"""\
                #!/usr/bin/env python3
                import rclpy

                def main():
                    rclpy.init()
                    # TODO: implement {mod}
                    rclpy.shutdown()
                """))
            os.chmod(node_path, 0o755)

            # Pytest stub
            test_py = base/"test"/f"test_{mod}.py"
            test_py.write_text(textwrap.dedent(f"""\
                import pytest
                from {pkg}.{mod} import main

                def test_{mod}_callable():
                    assert callable(main)
                """))

        # setup.py
        setup_py = textwrap.dedent(f"""\
            from setuptools import find_packages, setup

            package_name = '{pkg}'

            setup(
                name=package_name,
                version='0.0.0',
                packages=find_packages(exclude=['test']),
                data_files=[
                    ('share/ament_index/resource_index/packages',['resource/' + package_name]),
                    ('share/' + package_name,['package.xml']),
                    ('share/' + package_name + '/launch',['launch/{launch}.launch.py']),
                ],
                install_requires=['setuptools'],
                zip_safe=True,
                maintainer='{author}',
                maintainer_email='{email}',
                description='{desc}',
                license='MIT',
                tests_require=['pytest'],
                entry_points={{'console_scripts': [
        {'\n'.join(entry_pts)}
                ],}},
            )
        """)
        (base/"setup.py").write_text(setup_py)

        # setup.cfg
        setup_cfg = textwrap.dedent(f"""\
            [develop]
            script-dir=$base/lib/{pkg}
            [install]
            install-scripts=$base/lib/{pkg}
        """)
        (base/"setup.cfg").write_text(setup_cfg)

        # package.xml
        package_xml = textwrap.dedent(f"""\
            <?xml version="1.0"?>
            <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
            <package format="3">
              <name>{pkg}</name>
              <version>0.0.0</version>
              <description>{desc}</description>
              <maintainer email="{email}">{author}</maintainer>
              <license>MIT</license>

              <test_depend>ament_copyright</test_depend>
              <test_depend>ament_flake8</test_depend>
              <test_depend>ament_pep257</test_depend>
              <test_depend>python3-pytest</test_depend>

              <export>
                <build_type>ament_python</build_type>
              </export>
            </package>
        """)
        (base/"package.xml").write_text(package_xml)

        # launch file
        launch_py = textwrap.dedent(f"""\
            import launch
            from launch_ros.actions import Node

            def generate_launch_description():
                return launch.LaunchDescription([
        {''.join(f"        Node(package='{pkg}', executable='{n.replace('.py','')}', name='{n.replace('.py','')}_node'),\\n" for n in nodes)}
                ])
        """)
        (base/"launch"/f"{launch}.launch.py").write_text(launch_py)

        # README.md
        readme = textwrap.dedent(f"""\
            # {pkg}

            {purpose}

            **Author:** {author} <{email}>

            ## Description

            {desc}

            ## Nodes
        """)
        for n in nodes:
            readme += f"- `{n}`\n"
        (base/"README.md").write_text(readme)

        # .gitignore + git init
        gitignore = textwrap.dedent("""
            # ROS build artifacts
            build/
            install/
            log/

            # Python
            __pycache__/
            *.pyc
        """)
        (base/".gitignore").write_text(gitignore)
        subprocess.run(["git", "init"], cwd=str(base))

        self.button(self.FinishButton).setText("Done!")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    wiz = NodeMakerWizard()
    wiz.show()
    sys.exit(app.exec_())

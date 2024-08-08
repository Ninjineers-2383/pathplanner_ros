from typing import List
from xml.etree.ElementTree import ElementTree, Element, tostring, fromstring

base = """<root BTCPP_format="4">
    <BehaviorTree ID="{name}"></BehaviorTree>
</root>"""

class Command:
    xmlElement: Element

    def __init__(self, name: str, parameters: dict[str, str] | None = None):
        self.name = name
        self.xmlElement = Element(name, parameters)

class SequentialCommandGroup(Command):
    def __init__(self, commands: List[Command]):
        super().__init__('Sequence')
        self.xmlElement.extend(commands)

class ParallelCommandGroup(Command):
    def __init__(self, commands: List[Command]):
        super().__init__('ParallelAll', {'max_failures': '0'})
        self.xmlElement.extend(commands)

class NamedCommandTree:
    def __init__(self, ID: str, command: Command):
        self.xmlElement = fromstring(base.format(name=ID))
        self.xmlElement.find('/').append(command)
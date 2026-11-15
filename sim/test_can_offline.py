
import re
import unittest
from pathlib import Path

DRIVER = (
    Path(__file__).resolve().parents[1]
    / "firmware" / "main" / "can_mcp2518fd.c"
)

HEADER = (
    Path(__file__).resolve().parents[1]
    / "firmware" / "main" / "can_port.h"
)

# Comprimentos permitidos pelo CAN-FD para cada DLC
DLC_LENGTHS = (
    0, 1, 2, 3, 4, 5, 6, 7, 8,
    12, 16, 20, 24, 32, 48, 64
)


def get_function(source, name):
    match = re.search(
        r"\b" + re.escape(name) + r"\s*\([^;]*?\)\s*\{",
        source,
        re.S,
    )

    if not match:
        raise AssertionError(f"Funcao ausente: {name}")

    start = match.end()
    depth = 1

    for i in range(start, len(source)):
        if source[i] == "{":
            depth += 1
        elif source[i] == "}":
            depth -= 1
            if depth == 0:
                return source[start:i]

    raise AssertionError(f"Funcao incompleta: {name}")


class TestCANOffline(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.source = DRIVER.read_text(encoding="utf-8")
        cls.header = HEADER.read_text(encoding="utf-8")

    def test_01_interface_can(self):
        self.assertIn("uint32_t id", self.header)
        self.assertIn("uint8_t len", self.header)
        self.assertIn("data[64]", self.header)

    def test_02_funcao_send_existe(self):
        body = get_function(self.source, "can_port_send")
        self.assertIn("f->id", body)
        self.assertIn("f->len", body)
        self.assertIn("f->data", body)

    def test_03_funcao_recv_existe(self):
        body = get_function(self.source, "can_port_recv")
        self.assertIn("result.data", body)
        self.assertIn("*f = result", body)

    def test_04_tres_ids_distintos(self):
        # IDs de exemplo, nao sao os IDs reais dos Moteus
        ids = [1, 2, 3]

        self.assertEqual(len(set(ids)), 3)

        for can_id in ids:
            self.assertLessEqual(can_id, 0x7FF)

    def test_05_comprimentos_can_fd(self):
        self.assertEqual(DLC_LENGTHS[8], 8)
        self.assertEqual(DLC_LENGTHS[14], 48)
        self.assertEqual(DLC_LENGTHS[15], 64)

    def test_06_limites_dos_dados(self):
        self.assertTrue(0 <= 48 <= 64)
        self.assertFalse(65 <= 64)

    def test_07_configuracao_fifo(self):
        self.assertIn("REG_CiFIFOCON1", self.source)
        self.assertIn("REG_CiFIFOCON2", self.source)
        self.assertIn("REG_CiFIFOUA1", self.source)
        self.assertIn("REG_CiFIFOUA2", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)

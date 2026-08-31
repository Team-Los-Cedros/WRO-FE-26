import os
import shutil
import subprocess
import tempfile
import unittest
from pathlib import Path


RAIZ_RONDA = Path(__file__).resolve().parents[1]
SCRIPT_DEPLOY = RAIZ_RONDA / "deploy.sh"


def _encontrar_bash():
    encontrado = shutil.which("bash")
    if encontrado:
        return encontrado
    if os.name == "nt":
        for candidato in (
            Path("C:/Program Files/Git/bin/bash.exe"),
            Path("C:/Program Files/Git/usr/bin/bash.exe"),
        ):
            if candidato.is_file():
                return str(candidato)
    return None


def _ruta_para_bash(ruta):
    # Git Bash acepta C:/...; Linux recibe su ruta POSIX normal.
    return Path(ruta).resolve().as_posix()


class DeployNoDestructivoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.bash = _encontrar_bash()
        if cls.bash is None:
            raise unittest.SkipTest("bash no disponible para smoke test de deploy")

    def _ejecutar(self, *argumentos):
        return subprocess.run(
            [self.bash, _ruta_para_bash(SCRIPT_DEPLOY), *argumentos],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=30,
            check=False,
        )

    @staticmethod
    def _contenido(ruta):
        return {
            archivo.relative_to(ruta).as_posix(): archivo.read_bytes()
            for archivo in ruta.rglob("*")
            if archivo.is_file()
        }

    def test_preflight_no_crea_destino_y_deploy_no_sobrescribe(self):
        with tempfile.TemporaryDirectory(prefix="ronda-nueva-deploy-") as temporal:
            padre = Path(temporal)
            destino = padre / "instalacion nueva"
            destino_bash = _ruta_para_bash(destino)

            preflight = self._ejecutar("--dry-run", destino_bash)
            self.assertEqual(preflight.returncode, 0, preflight.stderr)
            self.assertFalse(destino.exists())
            self.assertIn("no se creo ni modifico", preflight.stdout)

            primero = self._ejecutar(destino_bash)
            self.assertEqual(primero.returncode, 0, primero.stderr)
            self.assertTrue((destino / "ronda_nueva" / "__init__.py").is_file())
            self.assertTrue((destino / "comun" / "__init__.py").is_file())
            self.assertTrue((destino / "requirements.txt").is_file())
            self.assertFalse((destino / "controlador_inicio.py").exists())
            self.assertFalse((destino / "ronda_camara").exists())

            testigo = destino / "ARCHIVO_EXISTENTE_NO_TOCAR.txt"
            testigo.write_text("intacto\n", encoding="utf-8")
            antes = self._contenido(destino)
            segundo = self._ejecutar(destino_bash)
            despues = self._contenido(destino)

            self.assertEqual(segundo.returncode, 3)
            self.assertEqual(antes, despues)
            self.assertIn("Destino existente", segundo.stderr)
            self.assertEqual(list(padre.iterdir()), [destino])


if __name__ == "__main__":
    unittest.main()

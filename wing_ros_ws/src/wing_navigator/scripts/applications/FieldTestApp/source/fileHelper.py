import sys
import traceback
from pathlib import Path
from PySide2.QtCore import QObject, Slot, QUrl

class FileHelper(QObject):
    def __init__(self, parent=None):
        super(FileHelper, self).__init__(parent)

    def _toLocalPath(self, fileUrlOrPath):
        if not fileUrlOrPath:
            return ""
        
        s = str(fileUrlOrPath)
        if s.lower().startswith("file:///"):
            s = s[7:]
        elif s.lower().startswith("file://"):
            s = s[5:]
        
        if sys.platform != "win32" and not s.startswith("/"):
            s = "/" + s
        if sys.platform == "win32" and s.startswith("/"):
            s = s[1:]
        
        print("[FileHelper] Converted Local Path:", s)
        return s

    @Slot(str, str, result=bool)
    def saveText(self, filePath, text):
        """Save text to filePath"""
        try:
            local = self._toLocalPath(filePath)
            p = Path(local)
            if not p.parent.exists():
                p.parent.mkdir(parents=True, exist_ok=True)
            p.write_text(text, encoding="utf-8")
            print("[FileHelper] saved", local)
            return True
        except Exception as e:
            print("[FileHelper] saveText error:", e)
            traceback.print_exc()
            return False

    @Slot(str, result=str)
    def loadText(self, filePath):
        """Load text from filePath"""
        try:
            local = self._toLocalPath(filePath)
            p = Path(local)
            content = p.read_text(encoding="utf-8")
            print("[FileHelper] loaded:", local)
            return content
        except Exception as e:
            print("[FileHelper] loadText error:", e)
            traceback.print_exc()
            return ""

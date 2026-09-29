from __future__ import annotations

import re
import unicodedata

_NON_ALNUM = re.compile(r"[^a-z0-9]+")


def normalize(text: str) -> str:
    """Minuscules, sans accents ni ponctuation : « L'Administration ! » -> « l administration »."""
    decomposed = unicodedata.normalize("NFKD", text.lower())
    stripped = "".join(c for c in decomposed if not unicodedata.combining(c))
    return _NON_ALNUM.sub(" ", stripped).strip() 

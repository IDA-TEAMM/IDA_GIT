"""JETSON TAKLİDİ — `backend="auto"` yolunu bu makinede CuPy'li gibi koştur.

Kullanım (pytest EKLENTİSİ olarak, otomatik yüklenmez):

    PYTHONPATH=.:prototype/tests python3 -m pytest prototype/tests/test_mppi.py \
        -p jetson_cupy_taklidi

⚠ TEZGÂH SINIRI: yalnız `_resolve_backend`'i yamalar, `_has_cupy()`'yi
yamalamaz. `_has_cupy()` ile korunan iki test (`..._falls_back_to_numpy...`,
`..._bit_identical_to_auto_on_cpu`) bu taklit altında GERÇEK Jetson'da
atlanacakken burada koşar ve kırmızı yanar — bu tezgâhın kusuru, kodun
değil. Gerçek Jetson'da o ikisi `pytest.skip` alır.

Orijinal amaç: pytest eklentisi: `backend="auto"` -> SAHTE CuPy (Jetson'daki hal).

Gerçek CuPy, bir cupy dizisiyle ham numpy dizisini karıştıran işlemde
`TypeError: Unsupported type <class 'numpy.ndarray'>` fırlatır. Burada o
sözleşme taklit edilir: numpy alt sınıfı, ufunc operandlarında DÜZ ndarray
görürse patlar. Böylece "hangi test Jetson'da kırmızı" sorusu bu makinede
ölçülebilir hale gelir.
"""
import numpy as np
import prototype.planning.mppi as M


class SahteCupyDizi(np.ndarray):
    def __array_ufunc__(self, ufunc, method, *girdi, **kw):
        for g in girdi:
            if type(g) is np.ndarray:            # düz ndarray = cihaz dışı
                raise TypeError(
                    f"Unsupported type {type(g)}")
        ham = [np.asarray(g).view(np.ndarray) if isinstance(g, np.ndarray) else g
               for g in girdi]
        cik = [np.asarray(v).view(np.ndarray) if isinstance(v, np.ndarray) else v
               for v in kw.pop("out", ())] or None
        if cik is not None:
            kw["out"] = tuple(cik)
        s = getattr(ufunc, method)(*ham, **kw)
        return s.view(SahteCupyDizi) if isinstance(s, np.ndarray) else s


class _SahteCupy:
    """numpy'yi vekilleyen sahte modül; diziler SahteCupyDizi olur."""
    def __getattr__(self, ad):
        gercek = getattr(np, ad)
        if not callable(gercek):
            return gercek
        def sarmal(*a, **kw):
            s = gercek(*a, **kw)
            return s.view(SahteCupyDizi) if isinstance(s, np.ndarray) else s
        return sarmal

    @staticmethod
    def asnumpy(a):
        return np.asarray(a).view(np.ndarray)


_ASIL = M._resolve_backend


def _taklit(backend):
    if backend == "auto":                        # Jetson'da cupy BULUNUR
        return _SahteCupy(), np.float32
    return _ASIL(backend)


def pytest_configure(config):
    M._resolve_backend = _taklit

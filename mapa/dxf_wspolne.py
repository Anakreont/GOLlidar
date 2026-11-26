"""Wspolne elementy plikow DXF: podzialka (warstwa SKALA) i przeliczanie jednostek przy pobieraniu."""
import io

JEDNOSTKI = {'m': (1.0, 6), 'cm': (100.0, 5), 'mm': (1000.0, 4)}     # mnoznik wzgledem metrow, kod $INSUNITS


def podzialka(msp, x, y, dl=5, warstwa='SKALA'):
    """Podzialka w metrach pod rysunkiem: odcinek dl m z kreskami co 1 m i opisami (wspolrzedne w metrach)."""
    h = 0.15
    msp.add_line((x, y), (x + dl, y), dxfattribs={'layer': warstwa})
    for i in range(dl + 1):
        msp.add_line((x + i, y), (x + i, y + (h if i % 5 else 2 * h)), dxfattribs={'layer': warstwa})
        msp.add_text(str(i), height=0.15, dxfattribs={'layer': warstwa}).set_placement((x + i - 0.05, y - 0.3))
    msp.add_text('m  (rysunek 1:1)', height=0.15, dxfattribs={'layer': warstwa}).set_placement((x + dl + 0.25, y - 0.3))


def warstwy(doc, lista):
    for n, kol in lista:
        if n not in doc.layers:
            doc.layers.add(n, color=kol)


def w_jednostkach(sciezka, jedn):
    """Zwraca bajty DXF (zapisanego w metrach) przeliczonego na jednostke 'm', 'cm' lub 'mm' - skala 1:1 w CAD."""
    import ezdxf
    from ezdxf.math import Matrix44
    s, kod = JEDNOSTKI[jedn]
    doc = ezdxf.readfile(sciezka)
    if s != 1.0:
        M = Matrix44.scale(s, s, s)
        for e in list(doc.modelspace()):
            try:
                e.transform(M)
            except (NotImplementedError, AttributeError):
                pass
    doc.header['$INSUNITS'] = kod
    doc.header['$MEASUREMENT'] = 1
    buf = io.StringIO()
    doc.write(buf)
    return buf.getvalue().encode(doc.output_encoding or 'utf-8', errors='replace')

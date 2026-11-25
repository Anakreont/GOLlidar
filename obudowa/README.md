# Obudowa głowicy skanera

Dwie części do druku 3D, bez podpór:

| Plik | Co to | Jak drukować |
|---|---|---|
| `podstawa.stl` | Pudełko 68 × 60 × 32 mm z gniazdem na nakrętkę 1/4" w dnie, słupkami pod BNO085 i wyjściami kabli USB | Dnem do stołu |
| `pokrywa.stl` | Płyta 68 × 60 × 3,6 mm, na której stoi LiDAR | Płasko |

Zalecane: PETG lub PLA, warstwa 0,2 mm, 3 obrysy, wypełnienie 30%.

## Elementy złączne

- nakrętka 1/4"-20 UNC (klucz 7/16", 11,1 mm) — 1 szt., wciskana w gniazdo w dnie od środka
- śruby M2 × 8–10 z nakrętkami — 3 szt. (LiDAR do pokrywy)
- wkręty M3 × 10 — 4 szt. (pokrywa do podstawy, wkręcane w otwory 2,7 mm)
- wkręty M2,5 × 6 — 2–4 szt. (BNO085 do słupków, otwory 2,1 mm)
- opaski zaciskowe 2,5 mm — do przypięcia adaptera USB LiDAR-u i konwertera UART (szczeliny w dnie)

## Montaż

1. Wciśnij nakrętkę 1/4" w sześciokątne gniazdo w dnie (kropla kleju, żeby nie wypadała).
2. Przykręć BNO085 do słupków, płasko, elementami do góry.
3. Adapter USB LiDAR-u i konwerter UART przypnij opaskami po prawej stronie; kable USB wyprowadź przez dwa wycięcia w ścianie.
4. LiDAR przykręć trzema śrubami M2 do pokrywy, złączem w stronę prostokątnego przepustu; przewód 4-pin przełóż przez przepust.
5. Przykręć pokrywę czterema wkrętami M3 i nakręć całość na gwint 1/4" monopodu.

## Zmiana wymiarów

Wszystkie wymiary są parametrami na początku `generuj.py` (`pip install trimesh manifold3d`, potem `python3 generuj.py`).
Rozstaw otworów BNO085 (12,7 × 20,32 mm) i wymiary płytek USB nie były sprawdzone na fizycznych elementach — zmierz przed drukiem.

# Tyczka w twardym plecaku (`tyczka.py`, rurka Ø40)

Rurka aluminiowa Ø40 × 2 mm (6060/6063), ok. 80 cm; kable idą w środku rurki (otwory Ø14–16 z przelotkami: ok. 5 cm pod korkiem i nad gniazdem w skrzyni) (kup 100 cm i przytnij po przymiarce: głowica LiDAR-u ok. 1,85 m nad podłogą).

| Plik | Co to | Ile | Elementy złączne |
|---|---|---|---|
| `korek.stl` | korek w górny koniec rurki, trzpień 1/4" do góry | 1 | śruba 1/4"-20 × 20–25 mm (łeb w gniazdo od dołu), śruba M4 × 50 + nakrętka jako kołek przez rurkę |
| `obejma.stl` | zacisk rurki na tylnej ścianie skrzyni | 2 | 4 × M5 × 20 przez ścianę + duże podkładki od zewnątrz, 2 × M4 × 20 + nakrętki do zacisku |
| `gniazdo.stl` | kubek na dnie skrzyni, rurka się w nim opiera | 1 | 4 × M5 × 16 przez dno + duże podkładki |

Wyjście rurki przez otwór Ø42 w górnej ściance (za rączką) z gumową przelotką. Kable przez dławnicę PG9/PG11 obok.
Przed drukiem zmierz rurkę suwmiarką i popraw `RURA_D`, `RURA_DW` w `tyczka.py`. Druk: PETG, 4 obrysy, wypełnienie 40 %.

## Kolano zamiast korka

`kolano_0.stl` (płyta pozioma – obecny LiDAR 2D) i `kolano_35.stl` (pochylone 35° – kopuła L2 do przodu). Inny kąt: `kolano(kat)` w `tyczka.py`.
Nasuwane na rurkę na 36 mm, kołek M4 × 60 przez kolano i rurkę. W górnej płycie śruba 1/4"-20 × 16 mm łbem od środka (wkładana przez okno z tyłu), na nią głowica.
Kable z rurki wychodzą oknem z tyłu (Ø18). Druk tuleją w dół; sufit kanału to most ok. 32 mm – bez podpór zwykle wychodzi, ewentualnie podpory tylko w kanale.

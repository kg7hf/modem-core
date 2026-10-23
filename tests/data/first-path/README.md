# The Part 5 recordings

Part 5 of the article series, "The first path", runs its walkthrough and its recording tests on
six WAV files: the "Hi" burst sent through the two-path channels of MIL-STD-188-110B Table XX.
The recordings are not in the repository. They are a release asset of it, and this folder holds
what Git keeps of them:

| File | What it is |
|---|---|
| `pack.json` | What to fetch: the release asset's URL, its size and SHA-256, the folder it unpacks to, and the SHA-256 of `cases.json`. |
| `cases.json` | The six cases: each recording's mode, channel, timeline and the expectations the tests check. The same bytes are in the package. |

## Fetching them

Configure with the article data on:

```bash
cmake -S . -B build -DMODEM_ARTICLE_DATA=ON
```

CMake downloads `testdata-first-path-v1.zip` from the release `testdata-first-path-v1`, checks its
size and SHA-256 against `pack.json`, unpacks it into `test-data/first-path-v1/` beside the source
and checks every file against the package's own `INDEX.sha256`. The archive stays in
`test-data/` too. Every build of the checkout uses that copy; it is not part of `build/`, and a
fresh build directory does not fetch it again. An ordinary configure downloads nothing.

A package that is there but fails a check stops the configure, with what failed. Remove
`test-data/first-path-v1/` and configure again: the kept archive unpacks again without the network.

## By hand

Download `testdata-first-path-v1.zip` from the repository's releases page, check that its size
and SHA-256 are the ones in `pack.json`, and unpack it so that `test-data/first-path-v1/VERSION`
exists. The next configure with `MODEM_ARTICLE_DATA=ON` checks it and uses it. To read a copy
unpacked somewhere else, `first_path_demo --data-dir <folder>` takes the package's folder.

## The package

The package's own `README.md` says what each recording shows, how it was made (the transmitter,
the channel simulator and its settings, the SNR calibration) and how it was checked. Each
recording's `.json` beside it gives its full provenance.

A published package is never changed. A change to the audio, the metadata or the expectations is
a new version, with a new release and a new `pack.json`.

The recordings are licensed GPL-3.0-or-later, like the rest of the repository.

# Rust package license supplements

These files fill the 19 missing notice entries found in
`artifacts/license-audit-batch-fix/dependencies.json`. They supplement, rather
than replace, licenses copied from downloaded crate archives.

`rust-supplements.json` maps every package name and version to its declared
license expression, official repository, exact `.cargo_vcs_info.json` commit,
workspace path, and downloaded source URLs. Each entry includes SHA-256 of every
preserved file. Each package directory also contains `SOURCE.json` and the
original Cargo VCS metadata. Shared workspace licenses are copied into each
affected package directory so that redistribution does not depend on symlinks.

The files were retrieved unmodified on 2026-10-17. Where the GitHub tree API was
available, Git blob SHA-1 was additionally checked against that commit's tree;
the manifest reports this separately from ordinary content hashing. An API rate
limit did not affect fixed-commit raw file downloads.

Eighteen packages have original repository license files. `hexf-parse 0.2.1`
is explicitly different: the complete upstream archive at commit
`4225763d744183d720f575ae96d04161b4d08ea0` contains no LICENSE, COPYING or NOTICE.
Its unchanged `parse/Cargo.toml` declares `CC0-1.0`; that declaration is retained
as `upstream-Cargo.toml`, alongside the Creative Commons official CC0 legal text
as `LICENSE-CC0-1.0.txt`. This is not represented as an upstream-authored LICENSE,
and no copyright owner, year or bespoke permission text has been invented.

The exact hexf upstream archive and repository tree responses used for the
inspection are retained in `artifacts/license-audit-batch-fix/` for local audit.
Distribution should include these supplement directories and manifest together
with the notices collected from the registry packages.

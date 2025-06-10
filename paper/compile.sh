#!/bin/bash

PDF_INFILE=paper.md
PDF_LOGO=logo.png
PDF_OUTFILE=paper.pdf
TEX_OUTFILE=paper.tex
PDF_TEMPLATE=latex.template
authors=$(ob-paper get ${PDF_INFILE} authors:name)
title=$(ob-paper get ${PDF_INFILE} title)
repo=$(ob-paper get ${PDF_INFILE} repo)
archive_doi=$(ob-paper get ${PDF_INFILE} archive_doi)
formatted_doi=$(ob-paper get ${PDF_INFILE} formatted_doi)
paper_url=$(ob-paper get ${PDF_INFILE} paper_url)
review_issue_url=$(ob-paper get ${PDF_INFILE} review_issue_url)
pandoc \
    -V paper_title="${title}" \
    -V footnote_paper_title="${title}" \
    -V citation_author="${authors}" \
    -V repository="${repo}" \
    -V archive_doi="${archive_doi}" \
    -V formatted_doi="${formatted_doi}" \
    -V paper_url="http://joss.theoj.org/papers/" \
    -V review_issue_url="https://github.com/openjournals/joss-reviews/issues/${issue}" \
    -V issue="${issue}" \
    -V volume="${vol}" \
    -V year="${year}" \
    -V submitted="${submitted}" \
    -V published="${accepted}" \
    -V page="${issue}" \
    -V graphics="true" \
    -V logo_path="${PDF_LOGO}" \
    -V geometry:margin=1in \
    --verbose \
    -o "${PDF_OUTFILE}" \
    --pdf-engine=xelatex \
    --filter /usr/bin/pandoc-citeproc "paper.md" \
    --from markdown+autolink_bare_uris \
    --template "${PDF_TEMPLATE}" \
    -s
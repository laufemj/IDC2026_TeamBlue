# Documentation

[Back to the project](../README.md)

## Start here

| Document | Use it for |
|---|---|
| [One-page award profile](portfolio/Minjoon_Jang_IDC2026_One_Page.pdf) | A quick overview of the award and Minjoon Jang's contribution |
| [Portfolio and certificate](portfolio/Minjoon_Jang_IDC2026_Portfolio.pdf) | Five pages of context, results and the original certificate |
| [Project overview](project-overview.md) | Architecture, competition results and lessons |
| [Getting started](getting-started.md) | Desktop checks, detector inference and deployment prerequisites |

## Technical references

- [Field specification](field_spec.md): rulebook extraction, measurements and provenance.
- [Shared field constants](../ball_color/field_spec.py): values consumed by the current code.
- [Detector evaluation](lr_field_report.html): real-photo analysis; download and open the HTML in a browser.
- [Model metadata](../models/README.md): checkpoint history, validation and exports.
- [Official rulebook](../Contest_Rules_final2026.pdf): normative competition rules.

## Historical recovery records

These records describe different project stages. Some commands target the
original Pi-only repository, which is not included in this checkout.

| Record | Scope |
|---|---|
| [Pi-session recovery](pi_session_recovery.md) | Recovered values, remaining gaps and superseded assumptions |
| [Field-day playbook](pi_field_day_playbook.md) | Setup and measurement notes; historical access credentials removed |
| [Sorter backlog](pi_sorter_backlog.md) | Historical field-day state and remaining work |
| [CAD brief](pi_session_ball_sorter_brief.txt) | Earlier hardware contract; its mission concept and geometry are not the final design |

## Reading the evidence

The PDF portfolio is a retrospective based on the team report and certificate.
Recovery notes recovered some hardware values, including motor GPIO pins, that
the PDF describes as lost. Consult the current code and recovery record for
the status of those values.

Competition behaviours, offline validation and current repository capabilities
are not interchangeable. The original deployed code was not retained, the exact
contest-day inference backend is uncertain, and recovered design stages can
differ from the final build. Historical notes remain available for comparison.

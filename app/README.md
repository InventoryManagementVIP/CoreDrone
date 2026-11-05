# app/: Companion App (reserved)

The Companion App (React + Supabase, with the LLM "inventory agent" and push notifications) is
built by a separate team in its **own repository**. Nothing in this repo depends on it.
TODO(app team): link to the app repository here.

This directory is reserved in case the app moves into the monorepo later
(docs/adr/0001-monorepo.md). Until then, the only coupling between the two teams is the data
contract in [`contracts/`](../contracts/README.md): the drone writes scan records in that format and
the app team owns the database behind it.

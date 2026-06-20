// ESLint flat config for the map viewer's TypeScript (assets/slam/*.ts).
//
// Robust, type-aware linting: ESLint core "recommended" plus typescript-eslint
// "recommended-type-checked" (which catches real bugs — floating promises,
// unsafe `any`, misused promises, await-thenable, …) using the project's
// tsconfig. Prettier's config is applied last so formatting rules are left to
// the prettier pre-commit hook (no rule conflicts). Run via `npm run lint` or
// the `eslint-map-viewer` pre-commit hook.
import eslint from "@eslint/js";
import tseslint from "typescript-eslint";
import globals from "globals";
import prettier from "eslint-config-prettier";

export default tseslint.config(
  { ignores: ["dist/", "node_modules/"] },
  {
    files: ["**/*.ts"],
    extends: [eslint.configs.recommended, ...tseslint.configs.recommendedTypeChecked],
    languageOptions: {
      parserOptions: {
        projectService: true,
        tsconfigRootDir: import.meta.dirname,
      },
      globals: { ...globals.browser },
    },
    rules: {
      // TypeScript (and the build-time tsc with noEmitOnError) already resolves
      // identifiers, so the core no-undef rule is redundant and would
      // false-positive on ambient DOM/library types.
      "no-undef": "off",
    },
  },
  prettier,
);

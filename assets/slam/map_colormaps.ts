// Colormaps for the `bagwiz map slam --viewer` viewer. Each is a 256-entry RGB
// lookup table generated from the matplotlib colormap of the same name (the same
// source as bagwiz's C++ color_mapper LUTs), so the viewer matches bagwiz's other
// colorizers. Tables are base64-encoded
// 256*3 bytes (R,G,B per stop) to keep this module small.
// Formatted by Prettier (pre-commit) and type-checked by tsc at build time.

const LUTS_B64: Record<string, string> = {
  turbo:
    "MBI7MhVDMxhKNBtRNR5YNiFfNyRmOCdtOSpzOi15Oy+APDKGPTWLPjiRPzuXPz6cQECiQUOnQUasQkmxQku1Q066RFG/RFTDRFbHRVnLRVzPRV7TRmHWRmTaRmbdRmngRmvjR27mR3HpR3PrR3buR3jwR3vyRn30RoD2RoL4RoX6Rof7RYr8RYz9RI/+Q5H+QpT/QZb/QJn/Ppv+PZ7+O6D9OqP8OKX7N6j6Nav4M633Ma/1L7L0LrTyLLfwKrnuKLzrJ77pJcDnI8PkIsXiIMffH8ndHsvaHM3YG9DVGtLSGtTQGdXNGNfKGNnIGNvFGN3CGN7AGOC9GeK7GeO5GuS2HOa0HeeyH+mvIOqsIuuqJeynJ+6kKu+hLPCeL/GbMvKYNfOUOPSRPPWOP/aKQ/eHRviESviATvl9Uvp6Vfp2WftzXfxvYfxsZf1paf1mbf5icf5fdf5cef5Zff9WgP9ThP9RiP9Oi/9Lj/9Jkv9Hlv5Emf5CnP5An/0/of09pPw8p/w6qfs5rPs4r/o3sfk2tPg2t/c1ufY1vPU0vvQ0wfM0w/E0xvA0yO80y+00zew00Oo00uk11Oc11+U12eQ22+I23eA339834d0349s45dk459c56dU569M57NE67s8678068cs68sk69Mc69cU69sM698E6+L45+bw5+ro5+7g4+7Y3/LM2/LE2/a41/aw0/qkz/qcy/qQx/qEw/p4v/pst/pks/pYr/pMq/pAp/Y0n/Yom/Icl/IQj+4Ei+34h+nsf+Xge+XUd+HIc928a9mwZ9WkY9GYX82MV8mAU8V0T8FsS71gR7VUQ7FMP61AO6k4N6EsM50kM5UcL5EUK4kMK4UEJ3z8I3T0I3DsH2jkH2DcG1jUG1DMF0jEF0C8Fzi0EzCsEyioEyCgDxSYDwyUDwSMCviECvCACuR4Ctx0CtBsBshoBrxgBrBcBqRYBpxQBpBMBoRIBnhABmw8BmA4BlQ0BkgsBjgoBiwkCiAgChQcCgQYCfgUCegQD",
  viridis:
    "RAFURAJWRQRXRQVZRgdaRghcRgpdRgteRw1gRw5hRxBjRxFkRxNlSBRnSBZoSBdpSBhqSBpsSBttSBxuSB1vSB9wSCBxSCFzSCN0SCR1SCV2SCZ3SCh4SCl5Ryp6Ryx6Ry17Ry58Ry99RjB+RjJ+RjN/RjSARTWBRTeBRTiCRDmDRDqDRDuEQz2EQz6FQj+FQkCGQkGGQUKHQUSHQEWIQEaIP0eIP0iJPkmJPkqJPkyKPU2KPU6KPE+KPFCLO1GLO1KLOlOLOlSMOVWMOVaMOFiMOFmMN1qMN1uNNlyNNl2NNV6NNV+NNGCNNGGNM2KNM2ONMmSOMmWOMWaOMWeOMWiOMGmOMGqOL2uOL2yOLm2OLm6OLm+OLXCOLXGOLHGOLHKOLHOOK3SOK3WOKnaOKneOKniOKXmOKXqOKXuOKHyOKH2OJ36OJ3+OJ4COJoGOJoKOJoKOJYOOJYSOJYWOJIaOJIeOI4iOI4mOI4qNIouNIoyNIo2NIY6NIY+NIZCNIZGMIJKMIJKMIJOMH5SMH5WLH5aLH5eLH5iLH5mKH5qKHpuKHpyJHp2JH56JH5+IH6CIH6GIH6GHH6KHIKOGIKSGIaWFIaaFIqeFIqiEI6mDJKqDJauCJayCJq2BJ62BKK6AKa9/KrB/LLF+LbJ9LrN8L7R8MbV7MrZ6NLZ5Nbd5N7h4OLl3Orp2O7t1Pbx0P7xzQL1yQr5xRL9wRsBvSMFuSsFtTMJsTsNrUMRqUsVpVMVoVsZnWMdlWshkXMhjXsliYMpgY8tfZcteZ8xcac1bbM1abs5YcM9Xc9BWddBUd9FTetFRfNJQf9NOgdNNhNRLhtVJidVIi9ZGjtZFkNdDk9dBldhAmNg+m9k8ndk7oNo5oto3pds2qNs0qtwyrdwwsN0vst0ttd4ruN4put4ovd8mwN8lwt8jxeAhyOAgyuEfzeEd0OEc0uIb1eIa2OIZ2uMZ3eMY3+MY4uQY5eQZ5+QZ6uUa7OUb7+Uc8eUd9OYe9uYg+OYh++cj/ecl",
  inferno:
    "AAAEAQAFAQEGAQEIAgEKAgIMAgIOAwIQBAMSBAMUBQQXBgQZBwUbCAUdCQYfCgciCwckDAgmDQgpDgkrEAktEQowEgoyFAs0FQs3Fgs5GAw8GQw+GwxBHAxDHgxFHwxIIQxKIwxMJAxPJgxRKAtTKQtVKwtXLQtZLwpbMQpcMgpeNApfNglhOAliOQljOwlkPQllPglmQApnQgpoRApoRQppRwtqSQtqSgxrTAxrTQ1sTw1sUQ5sUg5tVA9tVQ9tVxBuWRBuWhFuXBJuXRJuXxNuYRNuYhRuZBVuZRVuZxZuaRZuahdubBhubRhubxlucRluchpudBpudRtudxxteBxteh1tfB1tfR5tfx5sgB9sgiBshCBrhSFrhyFriCJqiiJqjCNpjSNpjyRpkCVokiVokyZnlSZnlydmmCdmmihlmylknSlknypjoCpjoitioyxhpSxgpi1gqC5fqS5eqy9erTBdrjBcsDFbsTJaszJatDNZtjRYtzVXuTVWujZVvDdUvThTvzlSwDpRwTpQwztPxDxOxj1Nxz5MyD9LykBKy0FJzEJIzkNHz0RG0EVF0kZE00dD1EhC1UpB10s/2Ew+2U092k4821A73VE63lI431M34FU24VY14lc041kz5Fox5Vww5l0v514u6GAt6WEr6mMq62Qp62Yo7Gcm7Wkl7mok72wj724h8G8g8XEf8XMd8nQc83Yb83gZ9HkY9XsX9X0V9n4U9oAT94IS94QQ+IUP+IcO+IkM+YsL+YwK+Y4J+pAI+pIH+pQH+5YG+5cG+5kG+5sG+50H/J8H/KEI/KMJ/KUK/KYM/KgN/KoP/KwR/K4S/LAU/LIW/LQY+7Ya+7gd+7of+7wh+74j+sAm+sIo+sQq+sYt+ccv+cky+cs1+M03+M8699E999NA9tVD9tdG9dlJ9dtM9N1P9N9T9OFW8+Na8+Vd8uZh8uhl8upp8ext8e1x8e918fF58vJ98vSC8/WG8/aK9PiO9fmS9vqW+Pua+fyd+v2h/P+k",
  plasma:
    "DQiHEAeIEweJFgeKGQaMGwaNHQaOIAaPIgaQJAaRJgWRKAWSKgWTLAWULgWVLwWWMQWXMwWXNQSYNwSZOASaOgSaPASbPgScPwScQQSdQwOeRAOeRgOfSAOfSQOgSwOhTAKhTgKiUAKiUQKjUwKjVQKkVgGkWAGkWQGlWwGlXAGmXgGmYAGmYQCnYwCnZACnZgCnZwCoaQCoagCobACobgCobwCocQCocgGodAGodQGodwGoeAGoegKoewKofQOofgOogASogQSngwWnhAWnhgamhwemiAimigmliwqljQuljgykjw2kkQ6jkg+jlBCilRGhlhOhmBSgmRWfmhafnBeenRidnhmdoBqcoRuboh2aox6apR+ZpiCYpyGXqCKWqiOVqySUrCaUrSeTriiSsCmRsSqQsiuPsyyOtC6NtS+MtjCLtzGKuDKJujOIuzSIvDWHvTeGvjiFvzmEwDqDwTuCwjyBwz2AxD5/xUB+xkF9x0J8yEN7yUR6ykV6y0Z5zEd4zEl3zUp2zkt1z0x00E1z0U5y0k9x01Fx1FJw1VNv1VRu1lVt11Zs2Fdr2Vhq2lpq2ltp21xo3F1n3V5m3l9l3mFk32Jj4GNj4WRi4mVh4mZg42hf5Gle5Wpd5Wtd5mxc525b529a6HBZ6XFY6XJX6nRX63VW63ZV7HdU7XlT7XpS7ntR73xR735Q8H9P8IBO8YFN8YNM8oRL84VL84dK9IhJ9IlI9YtH9YxG9o1F9o9E95BE95FD95NC+JRB+JVA+Zc/+Zg++Zo++ps9+pw8+p47+586+6E5+6I4/KM4/KU3/KY2/Kg1/Kk0/asz/awz/a4y/a8x/bEw/bIv/bQv/bUu/rct/rgs/ros/rsr/r0q/r4q/sAp/cIp/cMo/cUn/cYn/cgn/com/csm/M0l/M4l/NAl/NIl+9Mk+9Uk+9ck+tgk+tok+dwk+d0l+N8l+OEl9+Il9+Ql9uYm9ugm9ekm9esn9O0n8+4n8/An8vIn8fQm8fUl8Pck8Pkh",
  jet: "AACAAACEAACJAACNAACSAACWAACbAACfAACkAACoAACtAACyAAC2AAC7AAC/AADEAADIAADNAADRAADWAADaAADfAADjAADoAADtAADxAAD2AAD6AAD/AAD/AAD/AAD/AAD/AAT/AAj/AAz/ABD/ABT/ABj/ABz/ACD/ACT/ACj/ACz/ADD/ADT/ADj/ADz/AED/AET/AEj/AEz/AFD/AFT/AFj/AFz/AGD/AGT/AGj/AGz/AHD/AHT/AHj/AHz/AID/AIT/AIj/AIz/AJD/AJT/AJj/AJz/AKD/AKT/AKj/AKz/ALD/ALT/ALj/ALz/AMD/AMT/AMj/AMz/AND/ANT/ANj/ANz+AOD7AOT4Auj0BuzxCfDuDPTrD/jnE/zkFv/hGf/eHP/bH//XI//UJv/RKf/OLP/KMP/HM//ENv/BOf++PP+6QP+3Q/+0Rv+xSf+tTf+qUP+nU/+kVv+gWv+dXf+aYP+XY/+UZv+Qav+Nbf+KcP+Hc/+Dd/+Aev99ff96gP93g/9zh/9wiv9tjf9qkP9mlP9jl/9gmv9dnf9aoP9WpP9Tp/9Qqv9Nrf9Jsf9GtP9Dt/9Auv88vv85wf82xP8zx/8wyv8szv8p0f8m1P8j1/8f2/8c3v8Z4f8W5P8T5/8P6/8M7v8J8fwG9PgC+PUA+/EA/u0A/+oA/+YA/+IA/94A/9sA/9cA/9MA/9AA/8wA/8gA/8QA/8EA/70A/7kA/7YA/7IA/64A/6sA/6cA/6MA/58A/5wA/5gA/5QA/5EA/40A/4kA/4YA/4IA/34A/3oA/3cA/3MA/28A/2wA/2gA/2QA/2AA/10A/1kA/1UA/1IA/04A/0oA/0cA/0MA/z8A/zsA/zgA/zQA/zAA/y0A/ykA/yUA/yIA/x4A/xoA/xYA/xMA+g8A9gsA8QgA7QQA6AAA5AAA3wAA2gAA1gAA0QAAzQAAyAAAxAAAvwAAuwAAtgAAsgAArQAAqAAApAAAnwAAmwAAlgAAkgAAjQAAiQAAhAAAgAAA",
  gray: "AAAAAQEBAgICAwMDBAQEBQUFBgYGBwcHCAgICQkJCgoKCwsLDAwMDQ0NDg4ODw8PEBAQEREREhISExMTFBQUFRUVFhYWFxcXGBgYGRkZGhoaGxsbHBwcHR0dHh4eHx8fICAgISEhIiIiIyMjJCQkJSUlJiYmJycnKCgoKSkpKioqKysrLCwsLS0tLi4uLy8vMDAwMTExMjIyMzMzNDQ0NTU1NjY2Nzc3ODg4OTk5Ojo6Ozs7PDw8PT09Pj4+Pz8/QEBAQUFBQkJCQ0NDRERERUVFRkZGR0dHSEhISUlJSkpKS0tLTExMTU1NTk5OT09PUFBQUVFRUlJSU1NTVFRUVVVVVlZWV1dXWFhYWVlZWlpaW1tbXFxcXV1dXl5eX19fYGBgYWFhYmJiY2NjZGRkZWVlZmZmZ2dnaGhoaWlpampqa2trbGxsbW1tbm5ub29vcHBwcXFxcnJyc3NzdHR0dXV1dnZ2d3d3eHh4eXl5enp6e3t7fHx8fX19fn5+f39/gICAgYGBgoKCg4ODhISEhYWFhoaGh4eHiIiIiYmJioqKi4uLjIyMjY2Njo6Oj4+PkJCQkZGRkpKSk5OTlJSUlZWVlpaWl5eXmJiYmZmZmpqam5ubnJycnZ2dnp6en5+foKCgoaGhoqKio6OjpKSkpaWlpqamp6enqKioqampqqqqq6urrKysra2trq6ur6+vsLCwsbGxsrKys7OztLS0tbW1tra2t7e3uLi4ubm5urq6u7u7vLy8vb29vr6+v7+/wMDAwcHBwsLCw8PDxMTExcXFxsbGx8fHyMjIycnJysrKy8vLzMzMzc3Nzs7Oz8/P0NDQ0dHR0tLS09PT1NTU1dXV1tbW19fX2NjY2dnZ2tra29vb3Nzc3d3d3t7e39/f4ODg4eHh4uLi4+Pj5OTk5eXl5ubm5+fn6Ojo6enp6urq6+vr7Ozs7e3t7u7u7+/v8PDw8fHx8vLy8/Pz9PT09fX19vb29/f3+Pj4+fn5+vr6+/v7/Pz8/f39/v7+////",
};

// Display order shown in the colormap dropdown. turbo is the default: high
// contrast that reveals structure in LiDAR maps while improving on jet.
export const COLORMAP_NAMES = ["turbo", "viridis", "inferno", "plasma", "jet", "gray"];
export const DEFAULT_COLORMAP = "turbo";

function decodeLut(b64: string): Uint8Array {
  const binary = atob(b64);
  const lut = new Uint8Array(binary.length);
  for (let i = 0; i < binary.length; i += 1) {
    lut[i] = binary.charCodeAt(i);
  }
  return lut; // 768 bytes: R,G,B per stop, 256 stops
}

const LUTS: Record<string, Uint8Array> = {};
for (const name of COLORMAP_NAMES) {
  LUTS[name] = decodeLut(LUTS_B64[name]);
}

// Sample colormap `name` at `t` (clamped to [0,1]), linearly interpolating
// between the two nearest of the 256 stops. Writes normalized r,g,b into
// out[off..off+2]. Unknown names fall back to the default map.
export function sampleColormap(
  name: string,
  t: number,
  out: Float32Array | number[],
  off: number,
): void {
  const lut = LUTS[name] || LUTS[DEFAULT_COLORMAP];
  const x = t <= 0 ? 0 : t >= 1 ? 1 : t;
  const pos = x * 255;
  const i0 = Math.floor(pos);
  const i1 = i0 >= 255 ? 255 : i0 + 1;
  const frac = pos - i0;
  const a = i0 * 3;
  const b = i1 * 3;
  out[off] = (lut[a] + (lut[b] - lut[a]) * frac) / 255;
  out[off + 1] = (lut[a + 1] + (lut[b + 1] - lut[a + 1]) * frac) / 255;
  out[off + 2] = (lut[a + 2] + (lut[b + 2] - lut[a + 2]) * frac) / 255;
}

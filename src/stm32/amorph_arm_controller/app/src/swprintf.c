
#include <stdarg.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>

/* 简单的窄字符格式化辅助：将 wchar_t 格式串按 ASCII 转换为 char 格式串，
   然后使用 vsnprintf 生成窄字符串，再转换回 wchar_t。此实现避免对宽字符库的依赖。
   仅适用于格式字符串本身为 ASCII，并且输出内容可用基本的 ASCII 表示。 */

int vswprintf(wchar_t *wcs, size_t maxlen, const wchar_t *wformat, va_list args) {
  if (!wcs || !wformat) return -1;

  /* 将宽字符格式串转换为窄字符格式串（仅 ASCII 有效） */
  enum { FMT_BUFSZ = 256 };
  char fmtbuf[FMT_BUFSZ];
  size_t i = 0;
  while (wformat[i] != (wchar_t)0 && i + 1 < FMT_BUFSZ) {
    const wchar_t wc = wformat[i];
    fmtbuf[i] = (wc >= 0 && wc <= 0x7F) ? (char)wc : '?';
    ++i;
  }
  fmtbuf[i] = '\0';

  /* 估算输出缓冲区大小，基于 maxlen，限制范围以避免大内存占用 */
  size_t out_cap;
  if (maxlen == 0) {
    out_cap = 256;
  } else {
    out_cap = maxlen * 4;
    if (out_cap < 256) out_cap = 256;
  }
  if (out_cap > 8192) out_cap = 8192;

  char *out = (char *)malloc(out_cap);
  if (!out) return -1;

  /* 使用 vsnprintf 生成窄字符输出 */
  va_list args_copy;
#if defined(__va_copy)
  __va_copy(args_copy, args);
#elif defined(va_copy)
  va_copy(args_copy, args);
#else
  /* 无法复制 va_list 的平台上直接使用 args（可能不可重入） */
  args_copy = args;
#endif
  int needed = vsnprintf(out, out_cap, fmtbuf, args_copy);
#if defined(va_copy) || defined(__va_copy)
  va_end(args_copy);
#endif

  if (needed < 0) {
    free(out);
    return needed;
  }

  /* 将窄字符串转换回 wchar_t 输出缓冲 */
  size_t produced = (size_t)needed;
  size_t copylen = produced;
  if (maxlen > 0) {
    if (copylen >= maxlen) copylen = maxlen - 1;
  }

  size_t j;
  for (j = 0; j < copylen; ++j) {
    unsigned char c = (unsigned char)out[j];
    wcs[j] = (wchar_t)c;
  }
  if (maxlen > 0) wcs[copylen] = (wchar_t)0;

  free(out);
  return (int)produced;
}

int swprintf(wchar_t *wcs, size_t maxlen, const wchar_t *wformat, ...) {
  int ret;
  va_list ap;
  va_start(ap, wformat);
  ret = vswprintf(wcs, maxlen, wformat, ap);
  va_end(ap);
  return ret;
}

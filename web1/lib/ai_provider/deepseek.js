/**
 * deepseek.js — DeepSeek / OpenAI 兼容端点 client
 *   - JSON mode (response_format: json_object) 强制结构化输出
 *   - 通过环境变量切换 provider:
 *       AI_BASE_URL=https://api.deepseek.com/v1     ← DeepSeek 默认
 *       AI_BASE_URL=https://api.openai.com/v1       ← OpenAI
 *       AI_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1  ← 通义
 */

const SYSTEM_PROMPT = `你是一位康复治疗师与运动医学专家,专门解读基于视觉动作捕捉的步态数据。
你的任务:根据给出的步态指标时间序列,输出一份客观、务实的步态评估报告。

严格遵守:
1. 使用中文输出。
2. 输出**必须**是合法 JSON,严格符合下方 schema,不要输出多余文字、markdown 代码围栏、注释。
3. summary 控制在 200 字以内;full 使用 markdown,500-800 字。
4. risks 数组每项 level 只能是 "low" | "medium" | "high" 三档,evidence 引用具体数值。
5. suggestions 提出**可执行的**训练/生活建议,忌泛泛而谈("多锻炼")或医学处方("服用XX")。
6. disclaimer 必须包含"本分析不构成医学诊断"。
7. **不要假设被观察对象的年龄段**。禁止使用"老人""老年人""长者""长辈""长辈们""爷爷奶奶"等年龄化称呼,统一用"被观察对象"或"使用者"或直接省略主语。评估维度只讨论观测到的步态数值本身,不基于年龄推断。
8. 步态指标口径:
   - spm(步频/每分钟步数):正常成人 100-120,助行器使用者 70-100 也可接受。
   - symmetry(左右对称性 %):>= 80 良好,60-80 轻度偏侧,< 60 明显偏侧。
   - kneeAmpL/R(左/右膝关节屈伸幅度,度):健康摆动相约 55-65°,< 30° 提示僵硬。
   - stability(稳定性 0-100):踝点垂直位移的规律性,>= 75 良好。
9. 可提及"步态特征提示卒中/帕金森等可能倾向"作为**倾向性判断**,但要明确"需专业医生评估"。

输出 JSON schema:
{
  "summary": string,              // 简要总评, <= 200 字
  "full": string,                 // 详细报告, markdown, 500-800 字, 结构:## 综合评估\\n\\n... \\n\\n## 逐项指标解读\\n... \\n\\n## 改进建议\\n...
  "risks": [
    { "name": string, "level": "low"|"medium"|"high", "evidence": string }
  ],                              // 一般给 3-5 项
  "suggestions": [ string ],      // 可执行建议列表, 3-5 条
  "disclaimer": string
}`;

function buildUserPrompt(metricsSeries, windowMinutes) {
  const series = metricsSeries.map(m => ({
    ts: new Date(m.ts).toISOString(),
    spm: m.spm,
    symmetry: m.symmetry,
    kneeAmpL: m.kneeAmpL,
    kneeAmpR: m.kneeAmpR,
    stability: m.stability,
  }));

  const summary = (() => {
    if (series.length === 0) return null;
    const avg = (k) => Math.round(series.reduce((s, x) => s + (x[k] ?? 0), 0) / series.length);
    return {
      avgSpm: avg('spm'),
      avgSym: avg('symmetry'),
      avgKneeL: avg('kneeAmpL'),
      avgKneeR: avg('kneeAmpR'),
      avgStab: avg('stability'),
      samples: series.length,
    };
  })();

  return `以下是被观察对象在最近 ${windowMinutes} 分钟内的步态指标数据(每 5 秒一采样):

采样统计:${JSON.stringify(summary)}

完整指标序列 (${series.length} 条):
${JSON.stringify(series, null, 0)}

请综合这些指标,输出符合 schema 的 JSON 报告。`;
}

/* 后处理: LLM 有概率违反 system prompt 的禁词, 再兜底过一遍.
 * 长词在前, 避免 "老年人" 被先切成 "老年" 后残留 "人". */
function sanitizeAgeTerms(text) {
  if (typeof text !== 'string') return text;
  return text
    .replace(/老年人们/g, '使用者')
    .replace(/老年人/g, '使用者')
    .replace(/老人们/g, '使用者')
    .replace(/老人/g, '使用者')
    .replace(/爷爷奶奶/g, '使用者')
    .replace(/爷爷/g, '使用者')
    .replace(/奶奶/g, '使用者')
    .replace(/长辈们/g, '使用者')
    .replace(/长辈/g, '使用者')
    .replace(/长者/g, '使用者')
    .replace(/老年/g, '');
}

function sanitizeReport(r) {
  if (!r || typeof r !== 'object') return r;
  const out = { ...r };
  out.summary    = sanitizeAgeTerms(out.summary);
  out.full       = sanitizeAgeTerms(out.full);
  out.disclaimer = sanitizeAgeTerms(out.disclaimer);
  if (Array.isArray(out.risks)) {
    out.risks = out.risks.map(x => ({
      ...x,
      name:     sanitizeAgeTerms(x.name),
      evidence: sanitizeAgeTerms(x.evidence),
    }));
  }
  if (Array.isArray(out.suggestions)) {
    out.suggestions = out.suggestions.map(sanitizeAgeTerms);
  }
  return out;
}

async function analyzeGait({ metricsSeries, windowMinutes, apiKey, model, baseUrl, timeoutMs }) {
  if (!apiKey) throw new Error('AI_API_KEY not set');

  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), timeoutMs || 30000);

  const url = baseUrl.replace(/\/$/, '') + '/chat/completions';
  const body = {
    model,
    messages: [
      { role: 'system', content: SYSTEM_PROMPT },
      { role: 'user', content: buildUserPrompt(metricsSeries, windowMinutes) },
    ],
    response_format: { type: 'json_object' },
    temperature: 0.3,
    max_tokens: 2000,
  };

  let resp;
  try {
    resp = await fetch(url, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'Authorization': `Bearer ${apiKey}`,
      },
      body: JSON.stringify(body),
      signal: controller.signal,
    });
  } catch (e) {
    clearTimeout(timer);
    if (e.name === 'AbortError') throw new Error(`AI request timeout after ${timeoutMs}ms`);
    throw new Error(`AI request failed: ${e.message}`);
  }
  clearTimeout(timer);

  if (!resp.ok) {
    const errText = await resp.text().catch(() => '');
    throw new Error(`AI API ${resp.status}: ${errText.slice(0, 400)}`);
  }

  const json = await resp.json();
  const content = json?.choices?.[0]?.message?.content;
  if (!content) throw new Error('AI response missing content');

  let parsed;
  try {
    parsed = JSON.parse(content);
  } catch (e) {
    throw new Error(`AI returned non-JSON content: ${content.slice(0, 200)}`);
  }

  const raw = {
    summary: parsed.summary || '',
    full: parsed.full || '',
    risks: Array.isArray(parsed.risks) ? parsed.risks : [],
    suggestions: Array.isArray(parsed.suggestions) ? parsed.suggestions : [],
    disclaimer: parsed.disclaimer || '本分析不构成医学诊断,如有异常请咨询专业医生。',
  };
  const cleaned = sanitizeReport(raw);
  return {
    ...cleaned,
    provider: 'deepseek',
    model,
    usage: json.usage || null,
  };
}

module.exports = { analyzeGait };

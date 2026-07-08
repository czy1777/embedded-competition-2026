/**
 * AI provider 抽象层
 *   根据 AI_PROVIDER 环境变量分发到具体实现,统一 analyzeGait() 接口。
 */

const mock = require('./mock');
const deepseek = require('./deepseek');

function analyzeGait({ metricsSeries, windowMinutes }) {
  const provider = (process.env.AI_PROVIDER || 'mock').toLowerCase();

  if (provider === 'mock') {
    return mock.analyzeGait({ metricsSeries, windowMinutes });
  }

  // deepseek / openai / 任何 OpenAI 兼容端点
  return deepseek.analyzeGait({
    metricsSeries,
    windowMinutes,
    apiKey: process.env.AI_API_KEY,
    model: process.env.AI_MODEL || 'deepseek-chat',
    baseUrl: process.env.AI_BASE_URL || 'https://api.deepseek.com/v1',
    timeoutMs: Number(process.env.AI_TIMEOUT_MS) || 30000,
  });
}

function currentProviderName() {
  return (process.env.AI_PROVIDER || 'mock').toLowerCase();
}

module.exports = { analyzeGait, currentProviderName };

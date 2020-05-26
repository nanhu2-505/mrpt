/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          https://www.mrpt.org/                         |
   |                                                                        |
   | Copyright (c) 2005-2020, Individual contributors, see AUTHORS file     |
   | See: https://www.mrpt.org/Authors - All rights reserved.               |
   | Released under BSD License. See: https://www.mrpt.org/License          |
   +------------------------------------------------------------------------+ */

#include "system-precomp.h"  // Precompiled headers

#include <mrpt/core/exceptions.h>
#include <mrpt/system/CControlledRateTimer.h>

using namespace mrpt::system;

CControlledRateTimer::CControlledRateTimer(const double rate_hz)
	: mrpt::system::COutputLogger("CControlledRateTimer")
{
	setRate(rate_hz);
}
void CControlledRateTimer::setRate(const double rate_hz)
{
	ASSERT_ABOVE_(rate_hz, 0.0);
	m_rate_hz = rate_hz;
	m_currentEstimatedRate = m_rate_hz;
}
bool CControlledRateTimer::sleep()
{
	xx;
	return true;
}
